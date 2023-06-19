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

#include <pangolin/display/image_view.h>
#include <pangolin/image/managed_image.h>

#include "rootba/bal/bal_problem.hpp"

namespace rootba {

class BalImageOverlay {
 public:
  struct Options {
    int min_image_size = 500;
    int max_image_size = 2000;
    double circle_radius = 3.0;
  };

  BalImageOverlay();

  void set_options(const Options& options);

  template <typename Scalar>
  void update(pangolin::ImageView& view, const BalProblem<Scalar>& bal_problem,
              FrameIdx frame_id);

  void draw() const;

 private:
  pangolin::ManagedImage<uint8_t> make_background_image(Vec2d image_size);

  Options options_;

  // for current image
  FrameIdx frame_id_;
  Vec2d image_size_;     //!< image size w/o border
  Vec2d center_;         //!< center of image w/ border
  double scale_factor_;  //!< scale factor for image coordindates

  std::vector<Vec2d> kpts_detected_;
  std::vector<Vec2d> kpts_projected_;
};

}  // namespace rootba
