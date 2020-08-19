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
#pragma once

#include "rootba/options/visitable_options.hpp"

namespace rootba {

/// Options related to loading a dataset and preprocessing it before
/// optimization (normalization, state perturbations, etc)
struct BalDatasetOptions : public VisitableOptions<BalDatasetOptions> {
  WISE_ENUM_CLASS_MEMBER(DatasetType, (AUTO, 0), ROOTBA, BAL, BUNDLER);

  BEGIN_VISITABLES(BalDatasetOptions);

  VISITABLE_META(std::string, input, help("input dataset file to load"));
  VISITABLE_META(DatasetType, input_type,
                 init(DatasetType::AUTO).help("type of dataset to load"));

  VISITABLE_META(bool, save_output,
                 init(false).help("save optimization result"));
  VISITABLE_META(
      std::string, output_optimized_path,
      init("optimized.cereal").help("output file for optimized problem"));

  VISITABLE_META(bool, normalize,
                 init(true).help("normalize the scale and global "
                                 "position of the BA map"));
  VISITABLE_META(double, normalization_scale, init(100));

  VISITABLE_META(double, rotation_sigma,
                 init(0).help("standard deviation of camera "
                              "rotation perturbation"));
  VISITABLE_META(double, translation_sigma,
                 init(0).help("standard deviation of camera "
                              "translation perturbation"));
  VISITABLE_META(double, point_sigma,
                 init(0).help("standard deviation of "
                              "point perturbation"));

  VISITABLE_META(
      int, random_seed,
      init(38401).help("Random seed used to set the state of the pseudo random "
                       "number generator used to generate the pertubations. If "
                       "the value is not positive, the seed is taken from a "
                       "random device."));

  VISITABLE_META(
      double, init_depth_threshold,
      init(0).help(
          "Threshold for filtering observations that are too close "
          "to the camera after loading the problem. (0 means no filtering)"));

  VISITABLE_META(bool, quiet,
                 init(false).help(
                     "if true, skip INFO level log output when loading data"));

  END_VISITABLES;
};

}  // namespace rootba
