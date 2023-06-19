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

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
PROJECT_DIR="$SCRIPT_DIR/.."

set -x
set -e

BUILD_TYPE="${1:-Release}"
if [ "$#" -ge 1 ]; then
   shift
fi

echo "$BUILD_TYPE build"

###############################################################################
## setup

# determine build dir
BUILD_DIR_SUFFIX=""
if [ -n "$ROOTBA_ARCH_SUFFIX" ]; then
    BUILD_DIR_SUFFIX=-$ROORTBA_ARCH_SUFFIX
    EXTRA_CMAKE_FLAGS="-DROOTBA_EXTERNAL_BUILD_DIR_SUFFIX=$BUILD_DIR_SUFFIX -DROOTBA_DEVELOPER_MODE=Off"
fi
BUILD_DIR="$PROJECT_DIR/build$BUILD_DIR_SUFFIX"

if [ `uname -s` == Darwin ]; then
    # macOS
    
    product_version=$(sw_vers -productVersion)
    os_vers=( ${product_version//./ } )
    os_vers_major="${os_vers[0]}"
    os_vers_minor="${os_vers[1]}"

    if [ $od_vers_major == 10 && $os_vers_minor -lt 15 ]; then
        echo "ERROR: macOS before 10.15 Catalina not supported"
    fi
else
    # Linux

    # determine correct default compiler

    # TODO: more flexibility when determining correct default compiler, depending on what is installed
    # GCC version 9 is minimum for decent C++17 support
    if [ -z "$CXX" ]; then
        for VER in 9 10 11; do
            if hash g++-$VER 2> /dev/null; then
                export CC=gcc-$VER
                export CXX=g++-$VER
                break
            fi
        done
    fi

    # if no GCC, try clang; minimum version is 9
    if [ -z "$CXX" ]; then
        for VER in 9 10 11 12; do
            if hash clang++-$VER 2> /dev/null; then
                export CC=clang-$VER
                export CXX=clang++-$VER
                break
            fi
        done
    fi
fi

# check that compiler can be found if set
if [ -n "$CXX" ]; then
   which "$CXX"
fi
if [ -n "$CC" ]; then
   which "$CC"
fi

if [ -n "$ROOTBA_CXXFLAGS" ]; then
    export CXXFLAGS="$CXXFLAGS $ROOTBA_CXXFLAGS"
fi

# number of logical cores on linux and macos
NUM_CORES=`(which nproc > /dev/null && nproc) || sysctl -n hw.logicalcpu || echo 1`
#NUM_CORES=1

# Prefer ninja if available
if which ninja; then
  EXTRA_CMAKE_FLAGS="${EXTRA_CMAKE_FLAGS} -GNinja"
else
  export MAKEFLAGS=-j$NUM_CORES
fi

#export VERBOSE=1

# NOTE: Newer versions of Clang / AppleClang support -march=native on Apple Silicon,
#       so at some point we can remove this workaround.
if [ $(uname -m) = "x86_64" ]; then
  CXX_MARCH="${CXX_MARCH:-native}"
fi

###############################################################################
## execute

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake $EXTRA_CMAKE_FLAGS -DCXX_MARCH="$CXX_MARCH" -DCMAKE_BUILD_TYPE="$BUILD_TYPE" "$@" ..
cmake --build .
