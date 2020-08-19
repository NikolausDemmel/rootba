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

#include "rootba/cli/bal_cli_utils.hpp"

#include <filesystem>
#include <iostream>

#include <clipp/clipp.h>
#include <nlohmann/json.hpp>

#include "rootba/bal/bal_app_options.hpp"
#include "rootba/cli/cli_options.hpp"
#include "rootba/util/format.hpp"

namespace rootba {

bool parse_bal_app_arguments(const std::string& application_summary, int argc,
                             char** argv, BalAppOptions& options) {
  using namespace clipp;
  using nlohmann::json;

  // command line parsing
  std::string working_dir;
  std::string config_path("rootba_config.toml");
  bool dump_config = false;
  json parsed_dataset_options = json::object();
  json parsed_solver_options = json::object();

  // prepare arguments
  auto cli =
      (option("-C", "--directory") &
           value("DIR", working_dir) %
               "Change to given directory before doing anything else.",
       option("--config") & value("PATH", config_path) % "path to config file",
       option("--dump-config").set(dump_config) %
           "print effective config and exit",
       (option("") % "dummy",
        cli_options(parsed_dataset_options, options.dataset, "")) %
           "dataset options",
       (option("") % "dummy",
        cli_options(parsed_solver_options, options.solver, "")) %
           "solver options");

  // Note: dummy options needed for help formatting workaround
  //       https://github.com/muellan/clipp/issues/58

  // parse arguments
  if (!parse(argc, argv, cli)) {
    auto executable_name = std::filesystem::path(argv[0]).filename();
    auto fmt = doc_formatting{}.doc_column(22);
    auto filter = param_filter{}.has_doc(tri::either);
    if (!application_summary.empty()) {
      std::cout << application_summary << "\n\n";
    }
    std::cout << "SYNOPSIS:\n"
              << usage_lines(cli, executable_name) << "\n\n"
              << "OPTIONS:\n"
              << documentation(cli, fmt, filter) << '\n';
    return false;
  }

  if (!working_dir.empty()) {
    std::filesystem::current_path(working_dir);
    LOG(INFO) << "Changed working directory to: {}"_format(
        std::filesystem::current_path().string());
  } else {
    LOG(INFO) << "Working directory is: {}"_format(
        std::filesystem::current_path().string());
  }

  // load config
  {
    OptionsInterface::LoadConfigOptions opt;
    opt.allow_unused_top_level = {"/batch_run", "/slurm"};
    opt.missing_config_file_is_error = false;
    CHECK(options._load(config_path, opt));
  }

  // override config with cli
  CHECK(options.dataset._load(parsed_dataset_options));
  CHECK(options.solver._load(parsed_solver_options));

  // print config
  if (dump_config) {
    LOG(INFO) << "Printing effective config.";
    options._print(std::cout);

    // exit application
    return false;
  }

  // parsing was successful
  return true;
}

}  // namespace rootba
