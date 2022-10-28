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

#include "rootba/ceres/bal_bundle_adjustment.hpp"

#include <ceres/ceres.h>

#include "rootba/bal/bal_pipeline_summary.hpp"
#include "rootba/ceres/ba_log_utils.hpp"
#include "rootba/ceres/bal_iteration_callback.hpp"
#include "rootba/ceres/bal_residuals.hpp"
#include "rootba/ceres/option_utils.hpp"

// #include "g2o/g2o/core/block_solver.h"
// #include "g2o/g2o/core/optimization_algorithm_levenberg.h"
// #include "g2o/g2o/solvers/linear_solver_eigen.h"
// #include "g2o/g2o/types/types_six_dof_expmap.h"
// #include "g2o/g2o/core/robust_kernel_impl.h"
// #include "g2o/g2o/solvers/linear_solver_dense.h"
// #include "g2o/g2o/types/types_seven_dof_expmap.h"

namespace rootba {

void bundle_adjust_ceres(BalProblem<double>& bal_problem,
                         const SolverOptions& solver_options, BaLog* log) {
  // options
  const bool projection_validitity_check =
      ceres_use_projection_validity_check(solver_options);

  // prepare camera state (parameters in BalProblem are not contiguous in
  // memory)
  constexpr auto CAM_STATE_SIZE = BalProblem<double>::CAM_STATE_SIZE;
  VecXd camera_state(bal_problem.num_cameras() * CAM_STATE_SIZE);
  bal_problem.copy_to_camera_state(camera_state);

  // manually define ordering for SC: lms 0, cams 1 (for large problems it can
  // sometimes be suboptimal if determined automatically)
  auto ordering = std::make_shared<ceres::ParameterBlockOrdering>();

  // setup camera parameter blocks
  ceres::Problem problem;
  ceres::LocalParameterization* camera_parameterization =
      new ceres::ProductParameterization(
          new ceres::EigenQuaternionParameterization(),
          new ceres::IdentityParameterization(6));
  for (int i = 0; i < bal_problem.num_cameras(); ++i) {
    double* cam_ptr = camera_state.data() + i * CAM_STATE_SIZE;
    problem.AddParameterBlock(cam_ptr, CAM_STATE_SIZE, camera_parameterization);
    ordering->AddElementToGroup(cam_ptr, 1);
  }

  // setup landmark parameter blocks and residuals
  ceres::LossFunction* loss_function =
      ceres_create_loss_function(solver_options.residual);
  for (auto& lm : bal_problem.landmarks()) {
    problem.AddParameterBlock(lm.p_w.data(), 3);
    ordering->AddElementToGroup(lm.p_w.data(), 0);

    for (const auto& [cam_idx, obs] : lm.obs) {
      ceres::CostFunction* cost =
          projection_validitity_check
              ? BalSnavelyReprojectionError<VALID_PROJECTIONS_ONLY>::create(
                    obs.pos)
              : BalSnavelyReprojectionError<>::create(obs.pos);
      problem.AddResidualBlock(cost, loss_function,
                               camera_state.data() + cam_idx * CAM_STATE_SIZE,
                               lm.p_w.data());
    }
  }

  // prepare options
  ceres::Solver::Options options;
  options.linear_solver_ordering = ordering;
  set_ceres_options(options, solver_options);

  // set iteration callback if logging
  std::unique_ptr<BalIterationCallback> callback;
  if (log != nullptr) {
    callback = std::make_unique<BalIterationCallback>(
        *log, bal_problem, camera_state, solver_options);
    options.callbacks.push_back(callback.get());
    options.update_state_every_iteration = true;
  }

  // run solver
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  print_ceres_summary(summary, solver_options);

  // update camera state
  bal_problem.copy_from_camera_state(camera_state);

  // log summary if logging
  if (log != nullptr) {
    log_ceres_summary(*log, "bal_ceres", summary);
    log->static_data.timing.optimize = summary.total_time_in_seconds;
    log->static_data.timing.update_total();
  }
}


// void bundle_adjust_g2o(BalProblem<double>& bal_problem,
//                        const SolverOptions& solver_options, BaLog* log) {
//   // options
//   const bool projection_validitity_check =
//       ceres_use_projection_validity_check(solver_options);

//   // Setup optimizer
//   g2o::SparseOptimizer optimizer;
//   g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

//   linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

//   g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

//   g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
//   optimizer.setAlgorithm(solver);
//   optimizer.setVerbose(true);

//   const rootba::BalProblem<double>::Cameras &cams = bal_problem.cameras();
//   const rootba::BalProblem<double>::Landmarks &landmarks = bal_problem.landmarks();

//   for (int i = 0; i < cams.size(); ++i) {
//     const rootba::BalProblem<double>::Camera &cam = cams.at(i);
//     g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
//     vSE3->setEstimate(g2o::SE3Quat(cam.T_c_w.so3().matrix(),cam.T_c_w.translation()));
//     vSE3->setId(i);
//     vSE3->setFixed(i==0);
//     optimizer.addVertex(vSE3);
//   }

//   for (int i = 0; i < landmarks.size(); ++i) {
//     const rootba::BalProblem<double>::Landmark &lm = landmarks.at(i);

//     g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
//     vPoint->setEstimate(lm.p_w);
//     int id = cams.size()+i;
//     vPoint->setId(id);
//     vPoint->setMarginalized(true);
//     optimizer.addVertex(vPoint);

//     for (const auto& [cam_idx, obs] : lm.obs) {
//       g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

//       e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
//       e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(cam_idx)));
//       e->setMeasurement(obs.pos);
//       e->setInformation(Eigen::Matrix2d::Identity());

//       g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
//       e->setRobustKernel(rk);
//       rk->setDelta(sqrt(5.991));

//       e->fx = 718.856;
//       e->fy = 718.856;
//       e->cx = 607.1928;
//       e->cy = 185.2157;
//       optimizer.addEdge(e);
//     }
//   }

//   optimizer.initializeOptimization();
//   optimizer.optimize(100);
//   using SO3 = Sophus::SO3<double>;

//   //Keyframes
//   // for (int i = 0; i < cams.size(); ++i) {
//   //     g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(i));
//   //     g2o::SE3Quat SE3quat = vSE3->estimate();

//   //     rootba::BalProblem<double>::Camera &cam = bal_problem.cameras().at(i);
//   //     cam.T_c_w.so3() = SO3(SE3quat.rotation());

//   //     cam.T_c_w.translation() = SE3quat.translation();
//   // }

//   // //Points
//   // for (int i = 0; i < landmarks.size(); ++i) {
//   //     g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(i+cams.size()));
//   //     bal_problem.landmarks().at(i).p_w = vPoint->estimate();
//   // }
// }
}  // namespace rootba
