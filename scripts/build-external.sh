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

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
PROJECT_DIR="$SCRIPT_DIR/.."

set -x
set -e


if [ `uname -s` == Darwin ]; then
    product_version=$(sw_vers -productVersion)
    os_vers=( ${product_version//./ } )
    os_vers_major="${os_vers[0]}"
    os_vers_minor="${os_vers[1]}"

    if [[ $os_vers_major == 10 && $os_vers_minor -lt 15 ]]; then
        echo "ERROR: macOS before 10.15 Catalina not supported"
    fi
else
    # enable sized deallocation Pangolin / pybind11 needs it for C++17
    # TODO: still needed??
#    EXTRA_CXX_FLAGS="-fsized-deallocation"

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

BUILD_TYPE="${1:-Release}"

# build all external dependencies with the same standard
# TODO: build all externals from withing cmake (external_project_add)
#     1. this ensures the actual correct parameters are used
#     2. it allows to use multiple parallel builds with different settings
ROOTBA_CXX_STANDARD=17

# number of logical cores on linux and macos
NUM_CORES=`(which nproc > /dev/null && nproc) || sysctl -n hw.logicalcpu || echo 1`
#NUM_CORES=1

NUM_PARALLEL_BUILDS=$NUM_CORES

EIGEN_DIR="$PROJECT_DIR/external/eigen"

# Important note on Eigen alignment and the arch flag. TLDR: Passing
# arch=native for all build types is currently the only viable option
# to avoid suble bugs with Eigen.
#
# Eigen uses 16 byte alignemnt by default, but switches to 32 byte
# alignment if AVX instructions are enabled. This is the case on
# modern Intel hardware if we pass arch=native. It is vital to ensure
# that all translation units, including all thirdparty libraries, use
# the same value for EIGEN_MAX_ALIGN_BYTES (see
# https://eigen.tuxfamily.org/dox/TopicPreprocessorDirectives.html),
# since the aliged-malloc functions provided by Eigen might not be
# inlined and thus be taken from any of the translation units. Our
# current approach is to ensure arch=native everywhere. A possible
# alternative would be to explicitly pass -DEIGEN_MAX_ALIGN_BYTES=32
# everywhere, then 32 bytes would be used regardless of whether avx is
# enabled or not.
#
# Note that Ceres might override with arch=native in Release mode no
# matter what we pass it and OpenGV overwrites to always build in
# release mode and use arch=native, so any other value for CXX_MARCH
# might not work as expected. Also, even though we explicitly pass
# -O3, it might be overwritten e.g. with -O2 if we don't also set the
# build type to Release.

CXX_MARCH="${CXX_MARCH:-native}"

EXTRA_CXX_FLAGS="$EXTRA_CXX_FLAGS -march=$CXX_MARCH"

# map ci-build types to Default with custom flags to avoid override of -g0
if [[ "${BUILD_TYPE}" == Ci* ]]; then
    if [[ "${BUILD_TYPE}" == CiRelWithDebInfo ]]; then
        EXTRA_CXX_FLAGS="$EXTRA_CXX_FLAGS -g0 -O3"
    elif [[ "${BUILD_TYPE}" == CiDebug ]]; then
        EXTRA_CXX_FLAGS="$EXTRA_CXX_FLAGS -g0"
    else
        echo "Unknown build type $BUILD_TYPE."
        exit 1
    fi
    BUILD_TYPE=Default
elif ! [[ "${BUILD_TYPE}" == *Debug ]]; then
    EXTRA_CXX_FLAGS="$EXTRA_CXX_FLAGS -O3"
fi

if [[ "${BUILD_TYPE}" == Sanitizer* ]]; then
    EXTRA_CXX_FLAGS="$EXTRA_CXX_FLAGS -g0 -fno-omit-frame-pointer -fsanitize=address"
    EXTRA_LINKER_FLAGS="$EXTRA_LINKER_FLAGS -fno-omit-frame-pointer -fsanitize=address"
    BUILD_TYPE=${BUILD_TYPE#Sanitizer}
fi

# TODO: check if needed
# avoid countless warnings on GCC9 (clang always outputs 4.2.1 for -dumpversion)
#if [ -n "$CXX" ] && $CXX -v 2>&1 | grep 'gcc version' > /dev/null; then
#    EXTRA_CXX_FLAGS="${EXTRA_CXX_FLAGS} -Wno-deprecated-copy"
#fi

# respect CXXFLAGS environment variable
EXTRA_CXX_FLAGS="${EXTRA_CXX_FLAGS} ${CXXFLAGS}"

EXTERNAL_BUILD_DIR_SUFFIX=""
if [ -n "$ROOTBA_ARCH_SUFFIX" ]; then
    EXTERNAL_BUILD_DIR_SUFFIX=-$ROOTBA_ARCH_SUFFIX
fi

cd "$PROJECT_DIR"

INSTALL_PREFIX="$PWD/external/install$EXTERNAL_BUILD_DIR_SUFFIX"
BUILD_EIGEN=external/build$EXTERNAL_BUILD_DIR_SUFFIX/eigen
BUILD_PANGOLIN=external/build$EXTERNAL_BUILD_DIR_SUFFIX/Pangolin
BUILD_CEREAL=external/build$EXTERNAL_BUILD_DIR_SUFFIX/cereal
BUILD_FMT=external/build$EXTERNAL_BUILD_DIR_SUFFIX/fmt
BUILD_ABSEIL=external/build$EXTERNAL_BUILD_DIR_SUFFIX/abseil-cpp
BUILD_CERES=external/build$EXTERNAL_BUILD_DIR_SUFFIX/ceres-solver

COMMON_CMAKE_ARGS=(
    -DCMAKE_C_COMPILER_LAUNCHER=ccache
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
    -DCMAKE_BUILD_TYPE=${BUILD_TYPE}
    -DCMAKE_CXX_FLAGS="$EXTRA_CXX_FLAGS"
    -DCMAKE_EXE_LINKER_FLAGS="$EXTRA_LINKER_FLAGS"
    -DCMAKE_SHARED_LINKER_FLAGS="$EXTRA_LINKER_FLAGS"
    -DCMAKE_CXX_STANDARD=$ROOTBA_CXX_STANDARD 
    -DCMAKE_CXX_EXTENSIONS=OFF
    -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON
    -DBUILD_SHARED_LIBS=OFF
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX"
    -DCMAKE_PREFIX_PATH="$INSTALL_PREFIX"
)


# TODO: configure shallow clone...
git submodule sync --recursive
git submodule update --init --recursive

#export VERBOSE=1

rm -rf "$INSTALL_PREFIX"
mkdir -p "$INSTALL_PREFIX"

##############################################
## Eigen
if true; then
#if false; then
rm -rf "$BUILD_EIGEN"
mkdir -p "$BUILD_EIGEN"
pushd "$BUILD_EIGEN"
cmake ../../eigen "${COMMON_CMAKE_ARGS[@]}" \
      -DBUILD_TESTING=OFF
make -j$NUM_PARALLEL_BUILDS
make install
popd
fi

##############################################
## Pangolin
if true; then
#if false; then
rm -rf "$BUILD_PANGOLIN"
mkdir -p "$BUILD_PANGOLIN"
pushd "$BUILD_PANGOLIN"
cmake ../../Pangolin "${COMMON_CMAKE_ARGS[@]}" \
    -DCMAKE_FIND_FRAMEWORK=LAST \
    -DEXPORT_Pangolin=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TOOLS=OFF \
    -DBUILD_TESTS=OFF \
    -DBUILD_PANGOLIN_PYTHON=OFF \
    -DBUILD_PANGOLIN_LIBOPENEXR=OFF
# Note: Now we install Eigen and it provides a config module
#    "-DEIGEN_INCLUDE_DIR=$EIGEN_DIR"
make -j$NUM_PARALLEL_BUILDS pangolin
make install
popd
fi

##############################################
## Cereal
if true; then
#if false; then
rm -rf "$BUILD_CEREAL"
mkdir -p "$BUILD_CEREAL"
pushd "$BUILD_CEREAL"
cmake ../../cereal "${COMMON_CMAKE_ARGS[@]}" \
    -DJUST_INSTALL_CEREAL=ON
make -j$NUM_PARALLEL_BUILDS
make install
popd
fi

##############################################
## {fmt}
if true; then
#if false; then
rm -rf "$BUILD_FMT"
mkdir -p "$BUILD_FMT"
pushd "$BUILD_FMT"
cmake ../../fmt "${COMMON_CMAKE_ARGS[@]}" \
    -DFMT_DOC=Off \
    -DFMT_TEST=Off
make -j$NUM_PARALLEL_BUILDS
make install
popd
fi

##############################################
## Abseil
if true; then
#if false; then
rm -rf "$BUILD_ABSEIL"
mkdir -p "$BUILD_ABSEIL"
pushd "$BUILD_ABSEIL"
cmake ../../abseil-cpp "${COMMON_CMAKE_ARGS[@]}"
make -j$NUM_PARALLEL_BUILDS
make install
popd
fi

##############################################
## Ceres
if true; then
#if false; then
rm -rf "$BUILD_CERES"
mkdir -p "$BUILD_CERES"
pushd "$BUILD_CERES"
cmake ../../ceres-solver "${COMMON_CMAKE_ARGS[@]}" \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTING=OFF \
    -DBUILD_BENCHMARKS=OFF \
    -DEXPORT_BUILD_DIR=OFF
make -j$NUM_PARALLEL_BUILDS ceres
make install
popd
fi
