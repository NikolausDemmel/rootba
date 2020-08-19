# Configuration

## Options

Options are defined in structs deriving from
[`VisitableOptions`](../src/rootba/options/visitable_options.hpp) and
use a reflection mechanism to automatically register loaders from
config file and command line, including conversion from string for
enum types.

The options structs are found in [`src/rootba/bal`](../src/rootba/bal):
- [`bal_app_options.hpp`](../src/rootba/bal/bal_app_options.hpp)
- [`bal_dataset_options.hpp`](../src/rootba/bal/bal_dataset_options.hpp)
- [`solver_options.hpp`](../src/rootba/bal/solver_options.hpp)
- [`bal_residual_options.hpp`](../src/rootba/bal/bal_residual_options.hpp)
- [`ba_log_options.hpp`](../src/rootba/bal/ba_log_options.hpp)

The implementation of the options system is found in
[`src/rootba/options`](../src/rootba/options).

## Command Line

All options can be set from the command line and passed arguments
override values in the config file.

There are three special command line options:
- `-C, --directory`: This allows to change the working directory
  before doing anything else (including loading the options file). All
  relative paths in other arguments or options are relative to the
  working directory. Output log files will (by default) be saved in
  the working directory.
- `--config`: This allows specifying the path to a config file
  (default: `rootba_config.toml`)
- `--dump-config`: This option prints the *effective* options
  (considering defaults, loaded config file, and command line
  arguments) and then immediately exits the application. You can for
  example use it to genereate a config file with all default options:
  ```
  ./bin/bal --dump-config --config /dev/null > rootba_config.toml
  ```

Here is the full help output:

```
$ ./bin/bal --help
Solve BAL problem with solver determined by config.

SYNOPSIS:
        bal [-C <DIR>] [--config <PATH>] [--dump-config]  [--input <STR>] [--input-type <ENUM>]
            [--save-output|--no-save-output] [--output-optimized-path <STR>]
            [--normalize|--no-normalize] [--normalization-scale <FLOAT>] [--rotation-sigma <FLOAT>]
            [--translation-sigma <FLOAT>] [--point-sigma <FLOAT>] [--random-seed <INT>]
            [--init-depth-threshold <FLOAT>] [--quiet|--no-quiet]  [--solver-type <ENUM>]
            [--verbosity-level <INT>] [--debug|--no-debug] [--num-threads <INT>]
            [--residual-robust-norm <ENUM>] [--residual-huber-parameter <FLOAT>] [--log-log-path
            <STR>] [--log-save-log-flags <FLAG>...] [--log-disable-all|--no-log-disable-all]
            [--optimized-cost <ENUM>] [--max-num-iterations <INT>] [--min-relative-decrease <FLOAT>]
            [--initial-trust-region-radius <FLOAT>] [--min-trust-region-radius <FLOAT>]
            [--max-trust-region-radius <FLOAT>] [--min-lm-diagonal <FLOAT>] [--max-lm-diagonal
            <FLOAT>] [--min-linear-solver-iterations <INT>] [--max-linear-solver-iterations <INT>]
            [--eta <FLOAT>] [--jacobi-scaling|--no-jacobi-scaling] [--jacobi-scaling-epsilon
            <FLOAT>] [--preconditioner-type <ENUM>] [--linear-solver-type <ENUM>]
            [--use-explicit-schur-complement|--no-use-explicit-schur-complement]
            [--function-tolerance <FLOAT>] [--gradient-tolerance <FLOAT>] [--parameter-tolerance
            <FLOAT>] [--check-gradients|--no-check-gradients] [--gradient-check-relative-precision
            <FLOAT>] [--gradient-check-numeric-derivative-relative-step-size <FLOAT>]
            [--use-double|--no-use-double]
            [--use-householder-marginalization|--no-use-householder-marginalization]
            [--staged-execution|--no-staged-execution] [--reduction-alg <INT>] [--initial-vee
            <FLOAT>] [--vee-factor <FLOAT>]

OPTIONS:
        -C, --directory
        <DIR>         Change to given directory before doing anything else.
        --config
        <PATH>        path to config file
        --dump-config print effective config and exit

        dataset options

            --input
            <STR>     input dataset file to load

            --input-type
            <ENUM>    type of dataset to load
                      (possible values: [AUTO, ROOTBA, BAL, BUNDLER])
            --save-output|--no-save-output
                      save optimization result

            --output-optimized-path
            <STR>     output file for optimized problem

            --normalize|--no-normalize
                      normalize the scale and global position of the BA map

            --normalization-scale
            <FLOAT>

            --rotation-sigma
            <FLOAT>   standard deviation of camera rotation perturbation

            --translation-sigma
            <FLOAT>   standard deviation of camera translation perturbation

            --point-sigma
            <FLOAT>   standard deviation of point perturbation

            --random-seed
            <INT>     Random seed used to set the state of the pseudo random number generator used
                      to generate the pertubations. If the value is not positive, the seed is taken
                      from a random device.

            --init-depth-threshold
            <FLOAT>   Threshold for filtering observations that are too close to the camera after
                      loading the problem. (0 means no filtering)

            --quiet|--no-quiet
                      if true, skip INFO level log output when loading data

        solver options

            --solver-type
            <ENUM>    Solver type; 'SQUARE_ROOT' for square root BA, 'SCHUR_COMPLEMENT' for
                      classical Schur complement BA, 'CERES' for a Ceres-based implementation.
                      (possible values: [SQUARE_ROOT, SCHUR_COMPLEMENT, CERES])
            --verbosity-level
            <INT>     Output verbosity level. 0: silent, 1: brief report (one line), 2: full report

            --debug|--no-debug
                      if true, print out additional info all around; may slow down runtime

            --num-threads
            <INT>     number of threads to use for optimization. 0 means the system should determine
                      a suitable number (e.g. number of virutal cores available)

            --residual-robust-norm
            <ENUM>    which robust norm to use. NONE: squared norm, HUBER: Huber norm.
                      (possible values: [NONE, HUBER])
            --residual-huber-parameter
            <FLOAT>   huber parameter for robust norm in pixels

            --log-log-path
            <STR>     path of BA log file

            --log-save-log-flags
            <FLAG>... which file types to save; can be 0 or multiple
                      (possible flags: [JSON, UBJSON])
            --log-disable-all|--no-log-disable-all
                      don't log anything; currently only affects the Ceres solver, where it skips
                      the custom iteration callback completely

            --optimized-cost
            <ENUM>    Which cost to consider for the 'cost decreased?' check in Levenberg-Marqardt.
                      ERROR considers all residuals, ERROR_VALID ignores residuals with negative z,
                      and ERROR_VALID_AVG compares the average of over valid residuals (only for
                      non-ceres solvers).
                      (possible values: [ERROR, ERROR_VALID, ERROR_VALID_AVG])
            --max-num-iterations
            <INT>     maximum number of solver iterations (0 means just initialize and exit)

            --min-relative-decrease
            <FLOAT>   Lower cound for the relative decrease before a step is accepted (see Ceres).

            --initial-trust-region-radius
            <FLOAT>   Determines the initial damping (see Ceres).

            --min-trust-region-radius
            <FLOAT>   Optimization terminates if the trust region radius becomes smaller than this
                      value (see Ceres).

            --max-trust-region-radius
            <FLOAT>   Defines the minimum damping we always add (see Ceres).

            --min-lm-diagonal
            <FLOAT>   Currently only affects Ceres.

            --max-lm-diagonal
            <FLOAT>   Currently only affects Ceres.

            --min-linear-solver-iterations
            <INT>     Minimum number of iterations for which the linear solver should run, even if
                      the convergence criterion is satisfied (see Ceres).

            --max-linear-solver-iterations
            <INT>     Maximum number of iterations for which the linear solver should run (see
                      Ceres).

            --eta
            <FLOAT>   Forcing sequence parameter. The truncated Newton solver uses this number to
                      control the relative accuracy with which the Newton step is computed. This
                      constant is passed to ConjugateGradientsSolver which uses it to terminate the
                      iterations when (Q_i - Q_{i-1})/Q_i < eta/i (see Ceres).

            --jacobi-scaling|--no-jacobi-scaling
                      Use Jacobian scaling (see Ceres); note that unlike Ceres, our manual solvers
                      consider the additional parameter jacobi_scaling_epsilon; moreover, the
                      recompute the scale in every iteration, where ceres computes it only once in
                      the beginning.

            --jacobi-scaling-epsilon
            <FLOAT>   additional option for manual solvers: use 1/(eps + norm(diag)) to scale
                      Jacobians; Ceres always uses eps == 1; a value 0 means 'floating point
                      epsilon' (different for float and double).

            --preconditioner-type
            <ENUM>    Which preconditioner to use for PCG (see Ceres). Valid values for QR solver:
                      JACOBI, SCHUR_JACOBI; valid values for SC solver: SCHUR_JACOBI; valid values
                      for Ceres: see Ceres.
                      (possible values: [IDENTITY, JACOBI, SCHUR_JACOBI, CLUSTER_JACOBI,
                      CLUSTER_TRIDIAGONAL])

            --linear-solver-type
            <ENUM>    linear solver type for Ceres. Manual solvers always use what corresponds to
                      'ITERATIVE_SCHUR' in Ceres.
                      (possible values: [DENSE_NORMAL_CHOLESKY, DENSE_QR, SPARSE_NORMAL_CHOLESKY,
                      DENSE_SCHUR, SPARSE_SCHUR, ITERATIVE_SCHUR, CGNR])

            --use-explicit-schur-complement|--no-use-explicit-schur-complement
                      only for Ceres

            --function-tolerance
            <FLOAT>   (new_cost - old_cost) < function_tolerance * old_cost; (see Ceres)

            --gradient-tolerance
            <FLOAT>   only for Ceres

            --parameter-tolerance
            <FLOAT>   only for Ceres

            --check-gradients|--no-check-gradients
                      only for Ceres

            --gradient-check-relative-precision
            <FLOAT>   only for Ceres

            --gradient-check-numeric-derivative-relative-step-size
            <FLOAT>   only for Ceres

            --use-double|--no-use-double
                      if false, use float instead of double (only manual)

            --use-householder-marginalization|--no-use-householder-marginalization
                      if true, use Householder instead of Givens to marginalize landmarks (only QR
                      solver)

            --staged-execution|--no-staged-execution
                      execute solver in stages rather than step by step (only QR solver)

            --reduction-alg
            <INT>     Reduction algorithm to use (only QR solver)

            --initial-vee
            <FLOAT>   initial decrease factor for trust region update during Levenberg-Marquardt;
                      Ceres uses fixed value of 2.0.

            --vee-factor
            <FLOAT>   update of decrease factor for trust region update during Levenberg-Marquardt;
                      Ceres uses fixed value of 2.0.
```


## Config File

The config file uses the [TOML](https://github.com/toml-lang/toml)
format and the default file name is `rootba_config.toml`. You can find
an example config files with all default values in
[`examples/config/rootba_config_default.toml`](../examples/config/rootba_config_default.toml).

> *Note:* When the implementation changes, the default config file can
> be updated with the following command:
> ```
> ./bin/bal --dump-config --config /dev/null > examples/config/rootba_config_default.toml
> ```

## CVPR'21 Solvers

The evaluated solvers in the CVPR'21 paper use the following
configuration.

All experiments from the paper have these options beyond default
values in common:

```
[dataset]
random_seed = 38401
rotation_sigma = 0
translation_sigma = 0.01
point_sigma = 0.01
init_depth_threshold = 0.1

[solver.residual]
robust_norm = "HUBER"

[solver.log]
save_log_flags = ["UBJSON"]
```

In addition to the common configuration, below are the solver-specific
options.

**rootba-32:**

```
[solver]
solver_type = "SQUARE_ROOT"
use_double = false
```

**rootba-64:**

```
[solver]
solver_type = "SQUARE_ROOT"
use_double = true
```

**explicit-32:**

```
[solver]
solver_type = "SCHUR_COMPLEMENT"
use_double = false
```

**explicit-64:**

```
[solver]
solver_type = "SCHUR_COMPLEMENT"
use_double = true
```

**ceres-implicit:**

```
[solver]
solver_type = "CERES"
linear_solver_type = "ITERATIVE_SCHUR"
use_explicit_schur_complement = false
```

**ceres-explicit:**

```
[solver]
solver_type = "CERES"
linear_solver_type = "ITERATIVE_SCHUR"
use_explicit_schur_complement = true
```
