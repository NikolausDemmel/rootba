#!/usr/bin/env bash
##
## BSD 3-Clause License
##
## This file is part of the RootBA project.
## https://github.com/NikolausDemmel/rootba
##
## Copyright (c) 2021, Nikolaus Demmel.
## All rights reserved.
##

# Update license headers in source files.

# Dependency: licenseheaders python package (install with pip)

# TODO: Make it also update C++ files automatically. (Consider files with multiple headers, e.g. SolverSummary.hpp)

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

DIRS=(
    "$SCRIPT_DIR/../python/"
    "$SCRIPT_DIR/../scripts"
)

YEAR=2021
OWNER="Nikolaus Demmel"
TEMPLATE="$SCRIPT_DIR/templates/license-py-sh.tmpl"

for d in "${DIRS[@]}"
do
    licenseheaders -d "$d" -y $YEAR -o "$OWNER" -t "$TEMPLATE" -vv
done

