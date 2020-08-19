/**
BSD 3-Clause License

This file is part of the RootBA project.
https://github.com/NikolausDemmel/rootba

Copyright (c) 2021, Nikolaus Demmel.
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
#include "rootba/qr/landmark_block.hpp"

#include "rootba/qr/landmark_block_dynamic.hpp"

#ifdef ROOTBA_INSTANTIATIONS_STATIC_LMB
#include "rootba/qr/landmark_block_static.hpp"
#endif

namespace rootba {

template <typename Scalar, int POSE_SIZE>
std::unique_ptr<LandmarkBlock<Scalar>>
LandmarkBlockFactory<Scalar, POSE_SIZE>::get_landmark_block(size_t obs_size) {
  std::unique_ptr<LandmarkBlock<Scalar>> lb;

  switch (obs_size) {
    case 0:
    case 1:
      LOG(FATAL) << "landmark.obs.size() " << obs_size << " is not supported.";
#ifdef ROOTBA_INSTANTIATIONS_STATIC_LMB
    case 2:
      lb.reset(new LandmarkBlockStatic<Scalar, POSE_SIZE, 2>);
      break;
    case 3:
      lb.reset(new LandmarkBlockStatic<Scalar, POSE_SIZE, 3>);
      break;
    case 4:
      lb.reset(new LandmarkBlockStatic<Scalar, POSE_SIZE, 4>);
      break;
    case 5:
      lb.reset(new LandmarkBlockStatic<Scalar, POSE_SIZE, 5>);
      break;
    case 6:
      lb.reset(new LandmarkBlockStatic<Scalar, POSE_SIZE, 6>);
      break;
    case 7:
      lb.reset(new LandmarkBlockStatic<Scalar, POSE_SIZE, 7>);
      break;
    case 8:
      lb.reset(new LandmarkBlockStatic<Scalar, POSE_SIZE, 8>);
      break;
#endif
    default:
      lb.reset(new LandmarkBlockDynamic<Scalar, POSE_SIZE>);
  }

  return lb;
}

#ifdef ROOTBA_INSTANTIATIONS_FLOAT
template class LandmarkBlockFactory<float, 9>;
#endif

#ifdef ROOTBA_INSTANTIATIONS_DOUBLE
template class LandmarkBlockFactory<double, 9>;
#endif

}  // namespace rootba
