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

# Tidy all source files in the project.
# Optionally take the build folder as argument; default is 'build' in the repsitory root.
#
# There is a clang-tidy bug that it doesn't find standard headers if
# the compiler set to cmake is a symlink from another directory. This
# happens for example when ccache executables are found on path. The
# solution is to make sure to pass the full compiler path to
# clang-tidy. Note that you just need to run `cmake`, you don't even
# need to compile the build folder.
#
# See also: https://bugs.llvm.org/show_bug.cgi?id=47460
#
# Example on macOS:
#
# mkdir build
# cd build
# CXX=/usr/bin/clang++ CC=/usr/bin/clang cmake ..
# cd ..
# ../scripts/clang-tidy-all.sh
#

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

BUILD_FOLDER="${1:-$SCRIPT_DIR/../build}"

CLANG_TIDY_COMMANDS="clang-tidy-17 clang-tidy-16 clang-tidy-15 clang-tidy-14 clang-tidy-13 clang-tidy-12 clang-tidy"
if [[ "$OSTYPE" == "darwin"* ]]; then
    CLANG_TIDY_COMMANDS="$CLANG_TIDY_COMMANDS $(brew --prefix llvm)/bin/clang-tidy"
fi

# find the first available command:
for CMD in $CLANG_TIDY_COMMANDS; do
    if hash $CMD 2>/dev/null; then
        CLANG_TIDY_CMD=$CMD
        break
    fi
done

if [ -z $CLANG_TIDY_CMD ]; then
    echo "clang-tidy not found..."
    exit 1
fi

# clang format check version
MAJOR_VERSION_NEEDED=12

MAJOR_VERSION_DETECTED=`$CLANG_TIDY_CMD -version | sed -n -E 's/.*version ([0-9]+).*/\1/p'`
if [ -z $MAJOR_VERSION_DETECTED ]; then
    echo "Failed to parse major version ($CLANG_TIDY_CMD):
`$CLANG_TIDY_CMD -version`"
    exit 1
fi

echo "clang-tidy version $MAJOR_VERSION_DETECTED ($CLANG_TIDY_CMD):
`$CLANG_TIDY_CMD -version`"

if [ $MAJOR_VERSION_DETECTED -lt $MAJOR_VERSION_NEEDED ]; then
    echo "Looks like your clang tidy is too old; need at least version $MAJOR_VERSION_NEEDED"
    exit 1
fi

# assume same location as clang-tidy
CLANG_APPLY_REPLACEMENTS_CMD="${CLANG_TIDY_CMD/clang-tidy/clang-apply-replacements}"
if ! hash "$CLANG_APPLY_REPLACEMENTS_CMD" 2>/dev/null; then
    echo "Didn't find clang-apply-replacments ($CLANG_APPLY_REPLACEMENTS_CMD)"
    exit 1
fi

# assume default locations of run-clang-tidy.py scripts
if [[ "$OSTYPE" == "darwin"* ]]; then
    # assume clang-tidy from homebrew installation
    RUN_CLANG_TIDY_CMD="$(brew --prefix llvm)"/share/clang/run-clang-tidy.py
    if [ ! -f "$RUN_CLANG_TIDY_CMD" ]; then
        # for newer llvm versions the script moved here:
        RUN_CLANG_TIDY_CMD="$(brew --prefix llvm)"/bin/run-clang-tidy
    fi
else
    RUN_CLANG_TIDY_CMD=/usr/lib/llvm-$MAJOR_VERSION_DETECTED/share/clang/run-clang-tidy.py
    if [ ! -f "$RUN_CLANG_TIDY_CMD" ]; then
        # for newer llvm versions the script moved here (on Ubuntu):
        RUN_CLANG_TIDY_CMD=/usr/bin/run-clang-tidy-$MAJOR_VERSION_DETECTED.py
    fi
fi
if ! hash "$RUN_CLANG_TIDY_CMD" 2>/dev/null; then
    echo "Didn't find run-clang-tidy.py ($RUN_CLANG_TIDY_CMD)"
    exit 1
fi

# run clang-tidy in parallel
"$RUN_CLANG_TIDY_CMD" \
    -clang-tidy-binary $CLANG_TIDY_CMD \
    -clang-apply-replacements-binary $CLANG_APPLY_REPLACEMENTS_CMD \
    -p $BUILD_FOLDER \
    -format \
    -j4
