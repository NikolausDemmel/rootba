#!/usr/bin/env bash
##
## BSD 3-Clause License
##
## This file is part of the RootBA project.
## https://github.com/NikolausDemmel/rootba
##
## Copyright (c) 2021-2023, Nikolaus Demmel.
## All rights reserved.
##


# Format all source files in the project.
# Optionally take folder as argument; default is full inlude and src dirs.

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

FOLDER="${1:-$SCRIPT_DIR/../src}"

CLANG_FORMAT_COMMANDS="clang-format-17 clang-format-16 clang-format-15 clang-format-14 clang-format-13 clang-format-12 clang-format-11 clang-format-10 clang-format"

# find the first available command:
for CMD in $CLANG_FORMAT_COMMANDS; do
    if hash $CMD 2>/dev/null; then
        CLANG_FORMAT_CMD=$CMD
        break
    fi
done

if [ -z $CLANG_FORMAT_CMD ]; then
    echo "clang-format not installed..."
    exit 1
fi

# clang format check version
MAJOR_VERSION_NEEDED=10

MAJOR_VERSION_DETECTED=`$CLANG_FORMAT_CMD -version | sed -n -E 's/.*version ([0-9]+).*/\1/p'`
if [ -z $MAJOR_VERSION_DETECTED ]; then
    echo "Failed to parse major version (`$CLANG_FORMAT_CMD -version`)"
    exit 1
fi

echo "clang-format version $MAJOR_VERSION_DETECTED (`$CLANG_FORMAT_CMD -version`)"

if [ $MAJOR_VERSION_DETECTED -lt $MAJOR_VERSION_NEEDED ]; then
    echo "Looks like your clang format is too old; need at least version $MAJOR_VERSION_NEEDED"
    exit 1
fi

find $FOLDER -iname "*.?pp" | xargs $CLANG_FORMAT_CMD -verbose -i
