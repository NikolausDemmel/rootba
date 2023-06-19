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

#include "rootba/pangolin/bal_image_overlay.hpp"

#include <pangolin/display/default_font.h>
#include <pangolin/gl/gldraw.h>

namespace rootba {

BalImageOverlay::BalImageOverlay() = default;

template <typename Scalar>
void BalImageOverlay::update(pangolin::ImageView& view,
                             const BalProblem<Scalar>& bal_problem,
                             FrameIdx frame_id) {
  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
  using Vec4 = Eigen::Matrix<Scalar, 4, 1>;

  frame_id_ = frame_id;

  // prepare storage
  const auto& lmdb = bal_problem.landmarks();
  const auto& target_cam = bal_problem.cameras().at(frame_id);
  kpts_detected_.clear();
  kpts_projected_.clear();
  image_size_.setZero();

  // compute projections and store observations
  for (const auto& lm : lmdb) {
    auto obs_it = lm.obs.find(frame_id);
    if (obs_it == lm.obs.end()) {
      continue;
    }

    kpts_detected_.emplace_back(obs_it->second.pos.template cast<double>());

    // projected position
    Vec4 p_cam = (target_cam.T_c_w * lm.p_w).homogeneous();
    Vec2 p_proj;
    target_cam.intrinsics.project(p_cam, p_proj);
    kpts_projected_.emplace_back(p_proj.template cast<double>());

    image_size_ = image_size_.cwiseMax(
        obs_it->second.pos.template cast<double>().cwiseAbs() * 2);
  }

  if (image_size_.minCoeff() < options_.min_image_size) {
    scale_factor_ = options_.min_image_size / image_size_.minCoeff();
  }
  if (image_size_.maxCoeff() * scale_factor_ > options_.max_image_size) {
    scale_factor_ = options_.max_image_size / image_size_.maxCoeff();
  }

  // update image view background outlining image bounds + compute center_
  auto img = make_background_image(image_size_);
  view.SetImage(img);
}

pangolin::ManagedImage<uint8_t> BalImageOverlay::make_background_image(
    Vec2d image_size) {
  image_size *= scale_factor_;

  Vec2d background_size = image_size.array() + 20;  // border offset
  center_ = background_size / 2;

  // create image
  pangolin::ManagedImage<uint8_t> img(background_size.x(), background_size.y());

  // fill whole image in light grey
  img.Fill(220);

  // fill subimage with darker grey
  Vec2d offset = (background_size - image_size) / 2;
  auto subimg =
      img.SubImage(offset.x(), offset.y(), image_size.x(), image_size.y());
  subimg.Fill(150);

  return img;
}

void BalImageOverlay::set_options(const Options& options) {
  options_ = options;
}

void BalImageOverlay::draw() const {
  glLineWidth(1.0);
  glEnable(GL_BLEND);

  // draw lines
  glColor3f(1.0, 0.0, 0.0);  // red
  for (size_t i = 0; i < kpts_detected_.size(); ++i) {
    pangolin::glDrawLine(kpts_detected_.at(i) * scale_factor_ + center_,
                         kpts_projected_.at(i) * scale_factor_ + center_);
  }

  // draw projected
  glColor3f(1.0, 0.0, 0.0);  // red
  for (const auto& pos : kpts_projected_) {
    pangolin::glDrawCirclePerimeter(pos * scale_factor_ + center_,
                                    options_.circle_radius);
  }

  // draw detected not hosted
  glColor3f(0.0, 0.0, 1.0);  // blue
  for (const auto& pos : kpts_detected_) {
    pangolin::glDrawCirclePerimeter(pos * scale_factor_ + center_,
                                    options_.circle_radius);
  }

  // Info text
  glColor3f(1.0, 0.0, 0.0);  // red
  pangolin::default_font()
      .Text("Keypoints: %d total", kpts_detected_.size())
      .Draw(12, 20);
}

template void BalImageOverlay::update(pangolin::ImageView& view,
                                      const BalProblem<double>& bal_problem,
                                      FrameIdx frame_id);

template void BalImageOverlay::update(pangolin::ImageView& view,
                                      const BalProblem<float>& bal_problem,
                                      FrameIdx frame_id);

}  // namespace rootba
