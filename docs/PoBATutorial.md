# Power Bundle Adjustment for Large-Scale 3D Reconstruction

This document provides a brief overview of how to build the project and reproduce the results in the [PoBA paper](https://arxiv.org/abs/2204.12834). For more detailed documentation about the implementation and tools in this repository, please see the [README of the RootBA project](../README.md).

The PoBA solver is integrated within the RootBA framework, which allows effortless and systematic comparison with other solvers. The framework offers batch evaluation and visualization tools to analyze performance differences across solvers. Additionally, we have implemented a power series preconditioner for the explicit Schur complement solver.

## Citation

If you find our work useful in your research, please consider citing:

```
@inproceedings{weber2023poba,
 author = {Simon Weber and Nikolaus Demmel and Tin Chon Chan and Daniel Cremers},
 title = {Power Bundle Adjustment for Large-Scale 3D Reconstruction},
 booktitle = {IEEE Conference on Computer Vision and Pattern Recognition (CVPR)},
 year = {2023}
}

@inproceedings{demmel2021rootba,
 author = {Nikolaus Demmel and Christiane Sommer and Daniel Cremers and Vladyslav Usenko},
 title = {Square Root Bundle Adjustment for Large-Scale Reconstruction},
 booktitle = {IEEE Conference on Computer Vision and Pattern Recognition (CVPR)},
 year = {2021}
}
```

## System requirements

The project supports recent versions of Ubuntu and macOS. To build the project, you will need the specified [toolchain](#toolchain) and [dependencies](#dependencies). Below, you will find instructions on how to install the dependencies on [Ubuntu](#installing-dependencies-on-ubuntu) or [macOS](#installing-depedencies-on-macos).

### OS
- Ubuntu 20.04 and newer
- macOS 10.15 "Catalina" and newer
    - Supported on both Intel- and ARM-based Macs

### Toolchain
- C++17 compiler
- [CMake](https://www.cmake.org/) 3.13 or newer

### Dependencies
The following dependencies are expected to be supplied externally,
e.g. from a system-wide install:
- [`TBB`](https://github.com/oneapi-src/oneTBB)
  > *Note:* You can control the location where TBB is found by setting
  > the environment variable `TBB_ROOT`, e.g. `export TBB_ROOT=/opt/intel/tbb`.
- [`glog`](https://github.com/google/glog)
- [BLAS](https://www.netlib.org/blas/) and
  [LAPACK](https://www.netlib.org/lapack/) routines are needed by
  `SuiteSparse`, and optionally used by Eigen and Ceres directly for
  some operations.
  > On `UNIX` OSes other than macOS we recommend
  > [ATLAS](http://math-atlas.sourceforge.net/), which includes `BLAS`
  > and `LAPACK` routines. It is also possible to use
  > [OpenBLAS](https://github.com/xianyi/OpenBLAS). However, one needs
  > to be careful to [turn off the
  > threading](https://github.com/xianyi/OpenBLAS/wiki/faq#wiki-multi-threaded)
  > inside `OpenBLAS` as it conflicts with use of threads in RootBA and
  > also Ceres. For example, export `OPENBLAS_NUM_THREADS=1`.
  > 
  > MacOS ships with an optimized `LAPACK` and `BLAS` implementation as
  > part of the `Accelerate` framework. The Ceres build system will
  > automatically detect and use it.
- Python3
    - Python dependencies are needed for scripts and tools to generate
config files, run experiments, plot results, etc.

### Installing dependencies on Ubuntu

**Toolchain and libraries**

```shell
# for RootBA and Ceres
sudo apt install \
    libgoogle-glog-dev \
    libgflags-dev \
    libtbb-dev \
    libatlas-base-dev \
    libsuitesparse-dev
# for Pangolin GUI
sudo apt install \
    libglew-dev \
    ffmpeg \
    libavcodec-dev \
    libavutil-dev \
    libavformat-dev \
    libswscale-dev \
    libavdevice-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff5-dev \
    libopenexr-dev
```

**Install CMake**

In case you don't have CMake installed, you can easily obtain and install a recent version using pip.

```shell
sudo apt install python3-pip
python3 -m pip install --user -U cmake

# put this in your .bashrc to ensure cmake from pip is found
export PATH="$HOME/.local/bin:$PATH"
```

**Dependenices for batch evaluation and visualization**

These dependencies are necessary for generating configs to solve multiple problems in a batch and for visualizing the results in a PDF.

```shell
python3 -m pip install --user -U py_ubjson matplotlib numpy munch scipy pylatex toml
sudo apt install texlive-latex-extra latexmk
```

### Installing depedencies on macOS

**Toolchain and libraries**

Install [Homebrew](https://brew.sh), then use it to install
dependencies:

```shell
brew install cmake glog gflags tbb suitesparse
brew install glew ffmpeg libjpeg libpng libtiff
```
**Dependenices for batch evaluation and visualization**

These dependencies are necessary for generating configs to solve multiple problems in a batch and for visualizing the results in a PDF.

```shell
brew install python bash gnu-getopt
python3 -m pip install --user -U py_ubjson matplotlib numpy munch scipy pylatex toml
brew install --cask mactex
```

## Building

Building process is divided into building the dependencies and RootBA. You can pass [CMake build type](https://cmake.org/cmake/help/latest/variable/CMAKE_BUILD_TYPE.html) to our building scripts by replacing `[BUILD_TYPE]`, default is `Release` if nothing is passed.

**Build dependencies and RootBA**

> NOTE: You can also build RootBA using the CMake standard workflow demonstrated [here](../README.md#building).

```shell
./scripts/build-external.sh [BUILD_TYPE]
./scripts/build-rootba.sh [BUILD_TYPE]
```

### Testing the build

Run an example problem to test if the project is successfully built.

```shell
./bin/bal --input data/rootba/bal/ladybug/problem-49-7776-pre.txt
```

It should produce a similar result as the following:

```
[...]

Iteration 20, error: 1.3660e+04 (mean res: 0.59, num: 31843), error valid: 1.3631e+04 (mean res: 0.59, num: 31812)
        [INFO] Stage 1 time 0.001s.
        [INFO] Stage 2 time 0.000s.
        [CG] Summary: Maximum number of iterations reached. Time 0.010s. Time per iteration 0.001s
        [EVAL] f_diff 1.0231e+01 l_diff 1.0244e+01 step_quality 9.9866e-01 ri1 1.3631e+04 ri2 1.3621e+04
        [Success] error: 1.3650e+04 (mean res: 0.59, num valid: 31812), lambda: 4.5e-13, cg_iter: 10, it_time: 0.014s, total_time: 0.322s
Final Cost: error: 1.3650e+04 (mean res: 0.59, num: 31843), error valid: 1.3621e+04 (mean res: 0.59, num: 31812)
NO_CONVERGENCE: Solver did not converge after maximum number of 20 iterations

[...]
```

## Running PoBA solver and preconditioner

To solve a single problem using either the PoBA solver or the PoBA preconditioner, you can specify the solver and preconditioner type.

```shell
# PoBA solver
./bin/bal --solver-type POWER_SCHUR_COMPLEMENT --input [PROBLEM]

# Explicit Schur complement solver with PoBA preconditioner
./bin/bal --solver-type SCHUR_COMPLEMENT --preconditioner-type POWER_SCHUR_COMPLEMENT --input [PROBLEM]
```

## Batch evaluation

The RootBA framework offers helpful scripts for performing systematic evaluations of multiple solvers on a dataset and visualizing the performance differences.

In this section, you will find a concise set of instructions that are essential for reproducing the experimental results in the PoBA paper. For more detailed documentation, please refer to the [Batch Evaluation Tutorial](./BatchEvaluationTutorial.md).

### Getting data

The solvers are evaluated on all 97 BAL bundle adjustment problems, which can be easily downloaded using our script. It downloads the problems to the expected location `../rootba_data/`.

```shell
./scripts/download-bal-problems.sh all
```

### Generating configurations

We use the `generate-batch-configs.py` script to systematically generate config files for evaluation. It expects `rootba_batch_config.toml` in the experiment folder.

```shell
mkdir ../rootba_experiments
cp examples/batch/poba_batch_config.toml ../rootba_experiments/rootba_batch_config.toml
./scripts/generate-batch-configs.py ../rootba_experiments/
```

The script generates a config for each experiment which will be run in the batch evaluation.

```shell
[...]
../rootba_experiments/cvpr_poba/4_rootba_poba_precond/rootba_config_it50_rootbasc_powersc_float_j04_final3068.toml
../rootba_experiments/cvpr_poba/4_rootba_poba_precond/rootba_config_it50_rootbasc_powersc_float_j04_final4585.toml
../rootba_experiments/cvpr_poba/4_rootba_poba_precond/rootba_config_it50_rootbasc_powersc_float_j04_final13682.toml
```

### Running experiments

We can start the batch evaluation once the data and configs are ready.

```shell
./scripts/run-all-in.sh ../rootba_experiments/cvpr_poba/
```

You can check the process in a new terminal.

```shell
./scripts/list-jobs.sh ../rootba_experiments/cvpr_poba/
```

### Visualizing the results

After the batch evaluation is complete, we use `generate-tables.py` to generate a PDF with tables and plots. It takes a `toml` file as input which defines which evaluations we want to visualize, and output a PDF file at `../rootba_experiments/tables/`.

```shell
cp examples/batch/experiments-poba.toml  ../rootba_experiments/
./scripts/generate-tables.py --config ../rootba_experiments/experiments-poba.toml
```

### Recommended config for testing

**Since we exhaustingly evaluated all the solvers on the whole BAL dataset under different settings, it can take days to reproduce the all the experiments in the paper.** For rapid prototyping and testing, we suggest the modifying the (copied) batch evaluation config [poba_batch_config.toml#L501-L511](../examples/batch/poba_batch_config.toml) to obtain a general understanding of the performance differences, but with significantly reduced the runtime.

- Reduce the number of Levenberg Marquardt iterations
  - `it50` -> `it20`
- Evaluate with double precision only
  - `all_float` -> `double`
- Use all CPU cores instead of 4 cores (testing machine config in the paper)
  - Remove `j04`
- Evaluate on a small subset of BAL dataset
  - `all_bal` -> `some_bal`

```
# RootBA QR solver
1_rootba_qr = ["it20", "rootbaqr", "double", "some_bal"]

# ceres solver (explicit and implicit)
2_rootba_ceres = ["it20", "ceres", "all_itsc", "some_bal"]

# PoBA solver
3_rootba_poba = ["it20", "rootbapower", "double", "maxorderpower20", "some_bal"]

# Schur complement solver with PoBA preconditioner (not presented in the paper)
4_rootba_poba_precond = ["it20", "rootbasc", "powersc", "double", "maxorderpower02", "some_bal"]
```


