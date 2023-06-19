#
# BSD 3-Clause License
#
# This file is part of the RootBA project.
# https://github.com/NikolausDemmel/rootba
#
# Copyright (c) 2021-2023, Nikolaus Demmel.
# All rights reserved.
#
import os
import re
from collections.abc import Mapping

from .log import load_ba_log

from .util import load_toml_if_exists
from .util import load_text_if_exists


class Run:
    """Loads files from a single run of an experiment from a folder (config, status, output, log, ...)
    
    A single run is one invocation of a solver with a specific config on a specific problem.
    This is meant to be used on directories created with the 'generate-batch-configs' and 'run-all-in' scripts.
    It's best-effort, loading as many of the files as are present.
    """

    def __init__(self, dirpath, seq_name_mapping):
        self.dirpath = dirpath

        self.config = load_toml_if_exists(os.path.join(dirpath, 'rootba_config.toml'))
        self.status = load_text_if_exists(os.path.join(dirpath, 'status.log'))
        self.output = load_text_if_exists(os.path.join(dirpath, 'slurm-output.log'))

        # if we have slurm output, it already contains the program output, so loading it would be redundant
        if self.output is None:
            self.output = load_text_if_exists(os.path.join(dirpath, 'output.log'))
        # backwards compatibility to older runs that had rootba-output.log instead of output.log
        if self.output is None:
            self.output = load_text_if_exists(os.path.join(dirpath, 'rootba-output.log'))

        self.log = load_ba_log(dirpath)

        self.seq_name = self._infer_sequence_name(self.log, dirpath, seq_name_mapping)

        print("loaded {} from '{}'".format(self.seq_name, dirpath))

    def is_ceres(self):
        return self.log.is_ceres()

    def is_failed(self):
        if self.log is None:
            return True

        if "Completed" not in self.status:
            return True

        return False

    def failure_str(self):
        if not self.is_failed():
            return ""
        if self.output:
            if "Some of your processes may have been killed by the cgroup out-of-memory handler" in self.output:
                return "OOM"
            if "DUE TO TIME LIMIT" in self.output:
                return "OOT"
        return "x"

    @staticmethod
    def _infer_sequence_name(log, dirpath, name_mapping):
        """Tries to infer the problem name from the log, or falls back to the parent folder name"""
        seq_name = ""
        try:
            path = log._static.problem_info.input_path
            m = re.match(r".*/bal/([^/]+)/problem-([0-9]+)-[^/]+.txt", path)
            if m:
                seq_name = "{}{}".format(m.group(1), m.group(2))
        except:
            pass

        # Fallback to detecting the sequence name base on the last component of the parent folder. This is intended
        # to work for run folders created with the 'generate-batch-configs' script, assuming the sequence is the
        # last component in '_batch.combinations'.
        if seq_name == "":
            seq_name = os.path.basename(dirpath).split("_")[-1]

        # optionally remap the sequence name to something else as defined in the experiments config
        if isinstance(name_mapping, Mapping) and seq_name in name_mapping:
            seq_name = name_mapping[seq_name]

        return seq_name

    @staticmethod
    def is_run_dir(dirpath):
        """Returns True if the folder may be a run directory, based on the present files
        
        This is intended to be used for auto-detecting run directories in a file tree.
        """

        files = ['status.log', 'slurm-output.log', 'output.log', 'ba_log.ubjson', 'ba_log.json']
        for f in files:
            if os.path.isfile(os.path.join(dirpath, f)):
                return True
        return False
