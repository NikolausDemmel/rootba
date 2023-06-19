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

#include "rootba/solver/solver_summary.hpp"

namespace rootba {

struct DatasetSummary {
  struct Stats {
    double mean = 0;
    double min = 0;
    double max = 0;
    double stddev = 0;
  };

  // type and source
  std::string type;
  std::string input_path;

  // basic problem info
  int num_cameras = 0;
  int num_landmarks = 0;
  int num_observations = 0;
  double rcs_sparsity = 0;

  // per landmark observation stats
  Stats per_lm_obs;

  // per host-frame landmark stats (for achored landmark representation)
  Stats per_host_lms;
};

struct PipelineTimingSummary {
  double load_time = 0;         // load from file
  double preprocess_time = 0;   // normalize, perturb, convert
  double optimize_time = 0;     // run bundle adjustment
  double postprocess_time = 0;  // save result
};

struct BalPipelineSummary {
  DatasetSummary dataset;
  PipelineTimingSummary timing;
  SolverSummary solver;
};

}  // namespace rootba
