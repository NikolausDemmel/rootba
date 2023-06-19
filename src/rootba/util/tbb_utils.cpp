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

#include "rootba/util/tbb_utils.hpp"

#include <atomic>
#include <thread>

#include <glog/logging.h>
#include <tbb/global_control.h>
#include <tbb/task_arena.h>
#include <tbb/task_scheduler_observer.h>

namespace rootba {

int hardware_concurrency() { return std::thread::hardware_concurrency(); }

int tbb_task_arena_max_concurrency() {
  return tbb::this_task_arena::max_concurrency();
}

int tbb_global_max_allowed_parallelism() {
  return tbb::global_control::active_value(
      tbb::global_control::max_allowed_parallelism);
}

int tbb_effective_max_concurrency() {
  return std::min(tbb_task_arena_max_concurrency(),
                  tbb_global_max_allowed_parallelism());
}

struct ScopedTbbThreadLimit::Impl {
  std::unique_ptr<tbb::global_control> tbb_global_control;
};

ScopedTbbThreadLimit::ScopedTbbThreadLimit(int num_threads)
    : impl_(new Impl()) {
  CHECK_GE(num_threads, 0);
  if (num_threads > 0) {
    // set number of threads for TBB
    // global thread limit is in effect until global_control object is destroyed
    // See also: https://link.springer.com/chapter/10.1007/978-1-4842-4398-5_11
    impl_->tbb_global_control = std::make_unique<tbb::global_control>(
        tbb::global_control::max_allowed_parallelism, num_threads);
  }
}

ScopedTbbThreadLimit::~ScopedTbbThreadLimit() { delete impl_; }

namespace {

// see: https://stackoverflow.com/a/24823749/1813258
class ConcurrencyTracker : public tbb::task_scheduler_observer {
  std::atomic<int> num_threads_;
  std::atomic<int> max_threads_;

 public:
  ConcurrencyTracker() : num_threads_(), max_threads_() { observe(true); }

  ~ConcurrencyTracker() override { observe(false); }

  void on_scheduler_entry(bool /*unused*/) override {
    int current_num = ++num_threads_;
    int current_max = max_threads_;
    while (current_max < current_num) {
      if (max_threads_.compare_exchange_weak(current_max, current_num)) {
        break;
      }
    }
  }
  void on_scheduler_exit(bool /*unused*/) override { --num_threads_; }

  int get_current_concurrency() { return num_threads_; }
  int get_peak_concurrency() { return max_threads_; }
};

}  // namespace

struct TbbConcurrencyObserver::Impl {
  ConcurrencyTracker tracker;
};

TbbConcurrencyObserver::TbbConcurrencyObserver() : impl_(new Impl()) {}

TbbConcurrencyObserver::~TbbConcurrencyObserver() { delete impl_; }

int TbbConcurrencyObserver::get_current_concurrency() const {
  return impl_->tracker.get_current_concurrency();
}

int TbbConcurrencyObserver::get_peak_concurrency() const {
  return impl_->tracker.get_peak_concurrency();
}

}  // namespace rootba
