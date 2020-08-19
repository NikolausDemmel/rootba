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

# Dependencies:
# pip3 install -U --user py_ubjson matplotlib numpy munch scipy pylatex toml

# also: latexmk and latex
#
# Ubuntu:
#     sudo apt install texlive-latex-extra latexmk

import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "python")))

import rootba.generate_tables

rootba.generate_tables.main()
