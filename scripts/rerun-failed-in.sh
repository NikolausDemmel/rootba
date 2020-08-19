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

#
# Usage:
#    rerun-failed-in.sh FOLDER
#
# Reruns all failed experiments that are found in a given folder.

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

"$SCRIPT_DIR"/list-jobs.sh "$1" -s -o failed | while read f; do
    echo "$f"
    "$SCRIPT_DIR"/rerun-one-in.sh "$f" || true
done
