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

#include <glog/logging.h>
#include <pangolin/display/image_view.h>
#include <pangolin/pangolin.h>

#include "rootba/bal/bal_app_options.hpp"
#include "rootba/bal/bal_problem.hpp"
#include "rootba/ceres/bal_bundle_adjustment.hpp"
#include "rootba/cli/bal_cli_utils.hpp"
#include "rootba/pangolin/bal_image_overlay.hpp"
#include "rootba/pangolin/bal_map_display.hpp"
#include "rootba/pangolin/gui_helpers.hpp"
#include "rootba/solver/bal_bundle_adjustment.hpp"
#include "rootba/util/format.hpp"

// Pangolin variables
constexpr int UI_WIDTH = 200;

using Button = pangolin::Var<std::function<void(void)>>;

int main(int argc, char** argv) {
  using namespace rootba;

  FLAGS_logtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  // parse cli and load config
  BalAppOptions options;
  if (!parse_bal_app_arguments("3D viewer for BAL problem.", argc, argv,
                               options)) {
    return 1;
  }

  // load data
  BalProblem<double> bal_problem =
      load_normalized_bal_problem<double>(options.dataset);

  // setup GUI variables and buttons
  bool bal_state_changed = true;
  pangolin::Var<int> show_frame("ui.show_frame", 0, 0, 1500);
  pangolin::Var<bool> show_image("ui.show_image", true, true);

  // Note: values used for visualizations in paper:
  //       point_size = 1, cam_weight = 1, cam_size = 0.5
  pangolin::Var<int> point_size("ui.point_size", 2, 1, 5);
  pangolin::Var<int> cam_weight("ui.cam_weight", 2, 1, 5);
  pangolin::Var<double> cam_size("ui.cam_size", 1.5, 0.5, 5);

  pangolin::Var<int> min_image_size("ui.min_image_size", 500, 100, 2000);
  pangolin::Var<int> max_image_size("ui.max_image_size", 2000, 100, 2000);
  pangolin::Var<double> circle_radius("ui.circle_radius", 3.0, 0.5, 20.0);

  Button optimize_ceres("ui.optimize", [&]() {
    // print options
    if (options.solver.verbosity_level >= 2) {
      LOG(INFO) << "Options:\n" << options;
    }

    if (options.solver.solver_type == SolverOptions::SolverType::CERES) {
      bundle_adjust_ceres(bal_problem, options.solver);
    } else {
      if (!options.solver.use_double) {
#ifdef ROOTBA_INSTANTIATIONS_FLOAT
        BalProblem<float> bal_problem_tmp = bal_problem.copy_cast<float>();
        bundle_adjust_manual(bal_problem_tmp, options.solver);
        bal_problem = bal_problem_tmp.copy_cast<double>();
#else
        LOG(FATAL) << "Compiled without float support.";
#endif
      } else {
#ifdef ROOTBA_INSTANTIATIONS_DOUBLE
        bundle_adjust_manual(bal_problem, options.solver);
#else
        LOG(FATAL) << "Compiled without double support.";
#endif
      }
    }
    bal_problem.postprocress(options.dataset);
    bal_state_changed = true;
  });
  pangolin::Var<int>::Attach("ui.max_num_iterations",
                             options.solver.max_num_iterations, 0, 100);
  pangolin::CreateWindowAndBind("BAL", 1800, 1000);

  glEnable(GL_DEPTH_TEST);

  pangolin::View& main_display = pangolin::CreateDisplay().SetBounds(
      0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0);

  pangolin::View& img_view_display = pangolin::CreateDisplay()
                                         .SetBounds(0.0, 1.0, 0.0, 0.3)
                                         .SetLayout(pangolin::LayoutEqual);

  pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0,
                                        pangolin::Attach::Pix(UI_WIDTH));

  // 3D visualization (initial camera view optimized to see full map)
  // BAL convention seems to be Y-axis up
  const double w = 640;
  const double h = 480;
  const double f = 400;
  const double initial_zoom = 20;
  pangolin::OpenGlRenderState camera_3d_display(
      pangolin::ProjectionMatrix(w, h, f, f, w / 2, h / 2, 1e-2, 1e5),
      pangolin::ModelViewLookAt(initial_zoom * -10, initial_zoom * 8,
                                initial_zoom * -10, 0, 0, 0, pangolin::AxisY));

  pangolin::View& display_3d =
      pangolin::Display("scene").SetAspect(-w / h).SetHandler(
          new pangolin::Handler3D(camera_3d_display));

  main_display.AddDisplay(display_3d);
  main_display.AddDisplay(img_view_display);

  rootba::BalMapDisplay map_display;

  BalImageOverlay image_overlay;
  pangolin::ImageView img_view;
  img_view.extern_draw_function = [&](pangolin::View& v) {
    image_overlay.draw();
  };
  img_view_display.AddDisplay(img_view);

  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    bool update_image_overlay = false;
    if (show_image.GuiChanged()) {
      img_view_display.Show(show_image);
      if (show_image) {
        update_image_overlay = true;
      }
    }

    if (show_frame.GuiChanged() || bal_state_changed) {
      // clip values
      show_frame.Meta().range[0] = 0;
      show_frame.Meta().range[1] = bal_problem.num_cameras() - 1;
      show_frame = std::min(show_frame.Get(), bal_problem.num_cameras() - 1);
      show_frame = std::max(show_frame.Get(), 0);

      update_image_overlay = true;
    }

    if (min_image_size.GuiChanged() || max_image_size.GuiChanged() ||
        circle_radius.GuiChanged()) {
      image_overlay.set_options(
          {min_image_size, max_image_size, circle_radius});
      update_image_overlay = true;
    }

    // update overlay
    if (update_image_overlay) {
      image_overlay.update(img_view, bal_problem, show_frame);
    }

    if (bal_state_changed) {
      map_display.update(bal_problem);
      bal_state_changed = false;
    }

    display_3d.Activate(camera_3d_display);
    glClearColor(1.0F, 1.0F, 1.0F, 1.0F);
    map_display.draw(show_frame, {point_size, cam_weight, cam_size});
    pangolin::glDrawAxis(Sophus::SE3d().matrix(), 10.0);

    img_view_display.Activate();

    pangolin::FinishFrame();
  }

  return 0;
}
