/**
BSD 3-Clause License

This file is part of the RootBA project.
https://github.com/NikolausDemmel/rootba

Copyright (c) 2021-2023, Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <basalt/utils/hash.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>

#include "rootba/cg/utils.hpp"
#include "rootba/util/assert.hpp"
#include "rootba/util/cast.hpp"

namespace rootba {

struct HashPair {
  template <class T1, class T2>
  inline size_t operator()(const std::pair<T1, T2>& v) const {
    size_t seed = 0;
    basalt::hash_combine(seed, v.first);
    basalt::hash_combine(seed, v.second);
    return seed;
  }
};

// map of camera index to block; used to represent sparse block diagonal matrix
template <typename Scalar>
using IndexedBlocks = tbb::concurrent_unordered_map<
    std::pair<size_t, size_t>,
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>, HashPair>;

// add diagonal to block-diagonal matrix; missing blocks are assumed to be 0
template <typename Scalar>
void add_diagonal(IndexedBlocks<Scalar>& block_diagonal, size_t num_blocks,
                  size_t block_size,
                  const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& diagonal) {
  ROOTBA_ASSERT(num_blocks * block_size == unsigned_cast(diagonal.size()));
  for (size_t idx = 0; idx < num_blocks; ++idx) {
    auto it = block_diagonal.find(std::make_pair(idx, idx));
    if (it == block_diagonal.end()) {
      block_diagonal.emplace_hint(
          it, std::make_pair(idx, idx),
          diagonal.segment(block_size * idx, block_size).asDiagonal());
    } else {
      it->second += diagonal.segment(block_size * idx, block_size).asDiagonal();
    }
  }
}

// scale dimensions of JTJ as you would do for jacobian scaling of J beforehand,
// with diagonal scaling matrix D: For jacobain we would use JD, so for JTJ we
// use DJTJD.
template <typename Scalar>
void scale_jacobians(
    IndexedBlocks<Scalar>& block_diagonal, size_t num_blocks, size_t block_size,
    const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& scaling_vector) {
  ROOTBA_ASSERT(num_blocks * block_size ==
                unsigned_cast(scaling_vector.size()));
  for (auto& [cam_idx, block] : block_diagonal) {
    auto d = scaling_vector.segment(block_size * cam_idx.first, block_size)
                 .asDiagonal();
    block = d * block * d;
  }
}

// sum up diagonal blocks in hash map for parallel reduction
template <typename Scalar>
class BlockDiagonalAccumulator {
 public:
  using VecX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
  using MatX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

  template <class Derived>
  inline void add(size_t idx, const Eigen::MatrixBase<Derived>& block) {
    auto it = block_diagonal.find(std::make_pair(idx, idx));
    if (it == block_diagonal.end()) {
      block_diagonal.emplace_hint(it, std::make_pair(idx, idx), block);
    } else {
      it->second += block;
    }
  }

  inline void add(size_t idx, MatX&& block) {
    auto it = block_diagonal.find(std::make_pair(idx, idx));
    if (it == block_diagonal.end()) {
      block_diagonal.emplace_hint(it, std::make_pair(idx, idx),
                                  std::move(block));
    } else {
      it->second += block;
    }
  }

  inline void add_diag(size_t num_blocks, size_t block_size,
                       const VecX& diagonal) {
    add_diagonal(block_diagonal, num_blocks, block_size, diagonal);
  }

  inline void join(BlockDiagonalAccumulator& b) {
    for (auto& [k, v] : b.block_diagonal) {
      auto it = block_diagonal.find(k);
      if (it == block_diagonal.end()) {
        block_diagonal.emplace_hint(it, k, std::move(v));
      } else {
        it->second += v;
      }
    }
  }

  IndexedBlocks<Scalar> block_diagonal;
};

// sum up diagonal blocks in hash map for parallel reduction
template <typename Scalar, int POSE_SIZE = 9>
class BlockSparseMatrix : public LinearOperator<Scalar> {
 public:
  using VecX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
  using MatX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

  BlockSparseMatrix(size_t rows, size_t cols) : rows(rows), cols(cols) {}

  inline void add(size_t x, size_t y, MatX&& block) {
    auto it = block_storage.find(std::make_pair(x, y));
    if (it == block_storage.end()) {
      block_storage.emplace_hint(it, std::make_pair(x, y), std::move(block));
    } else {
      it->second += block;
    }
  }

  inline void set_zero() {
    if (block_storage.empty()) {
      // workaround: avoid calling range() for empty container
      // see: https://github.com/oneapi-src/oneTBB/issues/641
      return;
    }

    auto body = [](const typename IndexedBlocks<Scalar>::range_type& range) {
      for (typename IndexedBlocks<Scalar>::iterator r = range.begin();
           r != range.end(); r++) {
        r->second.setZero(POSE_SIZE, POSE_SIZE);
      }
    };
    tbb::parallel_for(block_storage.range(), body);
  }

  inline void add_diag(size_t num_blocks, size_t block_size,
                       const VecX& diagonal) {
    add_diagonal(block_storage, num_blocks, block_size, diagonal);
  }

  inline void join(BlockSparseMatrix& b) {
    for (auto& [k, v] : b.block_storage) {
      auto it = block_storage.find(k);
      if (it == block_storage.end()) {
        block_storage.emplace_hint(it, k, std::move(v));
      } else {
        it->second += v;
      }
    }
  }

  VecX left_multiply(const VecX& x) const {
    ROOTBA_ASSERT(!cached_keys.empty());

    auto body = [&](const tbb::blocked_range<size_t>& range, VecX res) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        const auto& k = cached_keys[r];
        const auto& v = block_storage.at(k);
        const size_t i = k.first;
        const size_t j = k.second;

        res.template segment<POSE_SIZE>(j * POSE_SIZE) +=
            v.transpose() * x.template segment<POSE_SIZE>(i * POSE_SIZE);
      }
      return res;
    };

    tbb::blocked_range<size_t> range(0, cached_keys.size());

    // TODO: add proper size and pose size handling
    VecX identity = VecX::Zero(x.rows());

    VecX res = tbb::parallel_reduce(range, identity, body, std::plus<VecX>());
    return res;
  }

  VecX right_multiply(const VecX& x) const override {
    ROOTBA_ASSERT(!cached_keys.empty());

    auto body = [&](const tbb::blocked_range<size_t>& range, VecX res) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        const auto& k = cached_keys[r];
        const auto& v = block_storage.at(k);

        size_t i = k.first;
        size_t j = k.second;

        res.template segment<POSE_SIZE>(i * POSE_SIZE) +=
            v * x.template segment<POSE_SIZE>(j * POSE_SIZE);
      }
      return res;
    };

    tbb::blocked_range<size_t> range(0, cached_keys.size());

    // TODO: add proper size and pose size handling
    VecX identity = VecX::Zero(x.rows());

    VecX res = tbb::parallel_reduce(range, identity, body, std::plus<VecX>());
    return res;
  }

  size_t num_cols() const override { return cols; }

  inline void recompute_keys() const {
    cached_keys.clear();
    for (const auto& [k, v] : block_storage) {
      cached_keys.emplace_back(k);
    }
  }

  Eigen::SparseMatrix<Scalar, Eigen::RowMajor> get_sparse_matrix() const {
    // Since we know exactly the sparsity, we can do better than triplets:
    // https://stackoverflow.com/a/18160211/1813258
    // https://eigen.tuxfamily.org/dox/group__SparseQuickRefPage.html
    // But since it's for debugging, it probably doesn't matter...

    // prepare triplet vector
    size_t num_triplets = 0;
    for (const auto& [idx, mat] : block_storage) {
      num_triplets += mat.cols() * mat.rows();
    }
    std::vector<Eigen::Triplet<Scalar>> triplets;
    triplets.reserve(num_triplets);

    // generate all triplets
    for (const auto& [idx, mat] : block_storage) {
      const size_t row_offset = idx.first * POSE_SIZE;
      const size_t col_offset = idx.second * POSE_SIZE;

      for (Eigen::Index row = 0; row < mat.rows(); ++row) {
        for (Eigen::Index col = 0; col < mat.cols(); ++col) {
          triplets.emplace_back(row + row_offset, col + col_offset,
                                mat(row, col));
        }
      }
    }

    // build sparse matrix
    Eigen::SparseMatrix<Scalar, Eigen::RowMajor> res(rows, cols);
    if (!triplets.empty()) {
      res.setFromTriplets(triplets.begin(), triplets.end());
    }

    return res;
  }

  size_t rows, cols;

  IndexedBlocks<Scalar> block_storage;

  mutable std::vector<std::pair<size_t, size_t>> cached_keys;
};

}  // namespace rootba
