#!/usr/bin/env python3
#
# BSD 3-Clause License
#
# This file is part of the RootBA project.
# https://github.com/NikolausDemmel/rootba
#
# Copyright (c) 2021, Nikolaus Demmel.
# All rights reserved.
#

import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "python")))

import rootba.plot_logs

rootba.plot_logs.main()
