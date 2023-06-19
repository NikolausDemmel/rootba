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

#include <pangolin/gl/gl.h>

#include "rootba/bal/bal_problem.hpp"

namespace rootba {

class BalFrameDisplay;

class BalMapDisplay {
 public:
  struct Options {
    int point_size = 2;
    int cam_weight = 2;
    double cam_size = 1.5;
  };

  template <typename Scalar>
  void update(const BalProblem<Scalar>& bal_problem);

  void draw(FrameIdx selected_frame, const Options& options);

 private:
  // draw points from buffers
  void draw_pointcloud(bool selected, const Options& options);

  // update the gl buffers for the saved points if needed
  // returns true if actually updated
  bool update_buffers();

  bool need_buffer_update_ = true;

  std::vector<std::unique_ptr<BalFrameDisplay>> frames_;

  std::vector<Vec3f> points_;

  static constexpr int MAX_BUFFER_UPDATES = 100;

  static constexpr int INITIAL_BUFFER_ELEMENTS = 100;

  pangolin::GlSizeableBuffer vertex_buffer_{pangolin::GlArrayBuffer,
                                            INITIAL_BUFFER_ELEMENTS, GL_FLOAT,
                                            3, GL_DYNAMIC_DRAW};
};

class BalFrameDisplay {
 public:
  BalFrameDisplay(FrameIdx frame_id);

  // makes a copy of all points (so we can later re-render them with different
  // options) and updates buffers
  template <typename Scalar>
  void update(const BalProblem<Scalar>& bal_problem);

  // draw, updating gl buffers if needed
  void draw(bool selected, const BalMapDisplay::Options& options);

 private:
  // draw camera frustrum
  void draw_camera(bool selected, const BalMapDisplay::Options& options);

  const FrameIdx frame_id_;

  SE3d T_w_c_;
};

}  // namespace rootba
