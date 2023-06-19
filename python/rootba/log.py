#
# BSD 3-Clause License
#
# This file is part of the RootBA project.
# https://github.com/NikolausDemmel/rootba
#
# Copyright (c) 2021-2023, Nikolaus Demmel.
# All rights reserved.
#
import ubjson
import json
import os
import warnings

import numpy as np

from numbers import Number
from collections.abc import Mapping
from munch import Munch


class Log(Munch):
    """Extension of Munch to load a single ba log file
    
    The constructor expects a file path to the log file. If it ends with '.ubjson' it loads it using ubjson, else json. 
    It does some auto-conversion to recursively munchify and convert 1d and 2d lists of numbers into numpy arrays.    
    """

    def __init__(self, path):
        data = self._load(path)
        if data is None:
            Munch.__init__(self)
        else:
            Munch.__init__(self, data)

    @staticmethod
    def _load(path):

        if path.endswith("ubjson"):
            with open(path, 'rb') as f:
                data = ubjson.load(f)
        else:
            with open(path, 'r') as f:
                data = json.load(f)

        data = Log._convert(data)

        return data

    @staticmethod
    def _convert(data):

        if isinstance(data, Mapping):
            new_data = Munch()
            for k, v in data.items():
                if k.endswith("__values"):
                    continue  # skip; processed together with __index
                elif k.endswith("__index"):
                    idx = v
                    values = np.array(data[k.replace("__index", "__values")])
                    # convert to list of arrays according to start indices
                    res = np.split(values, idx[1:])
                    if all(len(res[0]) == len(x) for x in res):
                        res = np.array(res)
                    new_data[k.replace("__index", "")] = res
                else:
                    new_data[k] = Log._convert(v)
            return new_data
        elif isinstance(data, list) and len(data) > 0:
            if isinstance(data[0], Mapping):
                return type(data)(Log._convert(x) for x in data)
            elif isinstance(data[0], Number):
                return np.array(data)
            elif all(isinstance(x, list) for x in data):
                if len(data[0]) > 0 and isinstance(data[0][0], Number):
                    if all(len(x) == len(data[0]) for x in data[1:]):
                        # 2d array
                        return np.array(data)
                    elif all(len(x) == 0 for x in data[1:]):
                        # just first entry filled, all other 0 --> also return 2d array with 1 row
                        # (if instead we would return a 1d array, things end up being inconsistent for logs with just 1 iteration)
                        return np.array(data[0:1])
        else:
            return data


class BaLog(Log):
    """Extension generic Log class specifically for rootba ba_log files"""

    def __init__(self, path):
        Log.__init__(self, path)

        if '_type' not in self:
            warnings.warn("Loaded log file does not have expected '_type' key.")
        elif self._type != "rootba":
            warnings.warn("Loaded log file is not of expected type 'rootba', but {}.".format(self._type))

    def is_ceres(self):
        return self._static.solver.solver_type.endswith("_ceres")


def detect_log_filepath(dir, basename):

    for ext in ["ubjson", "json"]:
        path = os.path.join(dir, basename + "." + ext)
        if os.path.isfile(path):
            return path

    return None


def load_ba_log(dir, basename="ba_log"):
    """Load a ba_log file from a directory
    
    First looks for .ubjson file, then falls back to .json, and finally returns None of not found.
    """

    filepath = detect_log_filepath(dir, basename)

    if filepath is None:
        return None

    return BaLog(filepath)
