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

#include "rootba/pangolin/gui_helpers.hpp"

#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>

namespace rootba {

void render_camera(const Eigen::Matrix4d& T_w_c, float line_width,
                   const u_int8_t* color, float size_factor) {
  const double sz = size_factor;
  const int width = 640;
  const int height = 480;
  const float fx = 500;
  const float fy = 500;
  const float cx = 320;
  const float cy = 240;  // choose an arbitrary intrinsics because we don't need
                         // the camera be exactly same as the original one

  Eigen::Matrix3d Kinv;
  Kinv << 1 / fx, 0, -cx / fx, 0, 1 / fy, -cy / fy, 0, 0, 1;

  glColor3ubv(color);
  glLineWidth(line_width);
  pangolin::glDrawFrustum(Kinv, width, height, T_w_c, sz);
}

}  // namespace rootba
