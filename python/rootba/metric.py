#
# BSD 3-Clause License
#
# This file is part of the RootBA project.
# https://github.com/NikolausDemmel/rootba
#
# Copyright (c) 2021-2023, Nikolaus Demmel.
# All rights reserved.
#
import copy

import numpy as np


class ExperimentSpec:

    def __init__(self, string):
        if "@it" in string:
            self.name, it = string.split("@it")
            self.it = int(it)
        else:
            self.name = string
            self.it = -1

    def display_name(self, exp):
        if self.it == -1:
            return exp.display_name
        else:
            return "{} @ it{}".format(exp.display_name, self.it)


class Metric:

    def __init__(self,
                 display_name,
                 accessor,
                 decimals,
                 format_string="{:.{prec}f}",
                 highlight_top=True,
                 geometric_mean=False,
                 larger_is_better=False):
        self.display_name = display_name
        self.accessor = accessor
        self.decimals = decimals
        self.display_decimals = None
        self.relative_to_column = None
        self.relative_to_experiment = None
        self.relative_to_metric = None
        self.ratio = True
        self.format_string = format_string
        self.highlight_top = highlight_top
        self.larger_is_better = larger_is_better
        self.exclude_columns_highlight = []
        self.geometric_mean = geometric_mean
        self.failed_threshold = None

    def set_config(self, spec):
        # change defaults in case of "relative_to_..." display mode
        if any(k in spec for k in ["relative_to_column", "relative_to_experiment", "relative_to_metric"]):
            # maybe overwritten by explicit "decimals" / "format_string" below
            self.decimals = 3
            self.display_decimals = 3
            self.format_string = "{:.3f}"
            self.geometric_mean = True

        if "display_name" in spec:
            self.display_name = spec.display_name
        if "decimals" in spec:
            self.decimals = spec.decimals
        if "display_decimals" in spec:
            self.display_decimals = spec.display_decimals
        if "relative_to_column" in spec:
            self.relative_to_column = spec.relative_to_column
        if "relative_to_experiment" in spec:
            self.relative_to_experiment = ExperimentSpec(spec.relative_to_experiment)
        if "relative_to_metric" in spec:
            self.relative_to_metric = spec.relative_to_metric
        if "ratio" in spec:
            self.ratio = spec.ratio
        if "format_string" in spec:
            self.format_string = spec.format_string
        if "highlight_top" in spec:
            self.highlight_top = spec.highlight_top
        if "larger_is_better" in spec:
            self.larger_is_better = spec.larger_is_better
        if "exclude_columns_highlight" in spec:
            self.exclude_columns_highlight = spec.exclude_columns_highlight
        if "geometric_mean" in spec:
            self.geometric_mean = spec.geometric_mean
        if "failed_threshold" in spec:
            self.failed_threshold = spec.failed_threshold

    def effective_display_decimals(self):
        if self.display_decimals is not None:
            return self.display_decimals
        else:
            return self.decimals

    def get_value(self, exps, e, s, it):
        #try:
        value = self.accessor(e.runs[s].log, it)
        #except AttributeError as err:
        #    raise

        if self.relative_to_metric is not None:
            relative_to_metric_accessor = self.relative_to_metric.accessor
        else:
            relative_to_metric_accessor = self.accessor

        if self.relative_to_experiment is not None:
            relative_to_log = exps[self.relative_to_experiment.name].runs[s].log
            relative_to_it = self.relative_to_experiment.it
        else:
            relative_to_log = e.runs[s].log
            relative_to_it = it

        if self.relative_to_metric is not None or self.relative_to_experiment is not None:
            base_value = relative_to_metric_accessor(relative_to_log, relative_to_it)
            if self.ratio:
                value = value / base_value
            else:
                value = base_value - value
        return value


def get_linear_solver_time(l):
    if l.is_ceres():
        return l._static.solver.linear_solver_time_in_seconds
    else:
        #return l.stage2_time.sum() + l.step_solver_time.sum() + l.back_substitution_time.sum()
        return l.stage1_time.sum() + l.stage2_time.sum() + l.step_solver_time.sum() + l.back_substitution_time.sum()
        #return (l.scale_landmark_jacobian_time.sum() + l.perform_qr_time.sum() + l.stage2_time.sum() +
        #        l.step_solver_time.sum() + l.back_substitution_time.sum())


# yapf: disable
metric_desc = dict(
    # for rootba logs
    cost=Metric("cost", lambda l, it: l.cost[it], 3, format_string="{:.{prec}e}"),
    cost_valid=Metric("cost valid", lambda l, it: l.cost_valid[it], 3, format_string="{:.{prec}e}"),
    cost_avg_valid=Metric("cost avg valid", lambda l, it: l.cost_avg_valid[it], 3, format_string="{:.{prec}f}"),
    cost_valid_vs_ceres=Metric("cost valid vs ceres", lambda l, it: l.cost[it] if l.is_ceres() else l.cost_valid[it], 3, format_string="{:.{prec}e}"),
    num_it_total=Metric("#it", lambda l, it: l.iteration[it], 0),
    num_it_valid=Metric("#it valid", lambda l, it: sum(l.step_is_valid[1:]), 0),
    num_it_successful=Metric("#it succ", lambda l, it: sum(l.step_is_successful[1:]), 0),
    num_it_inner=Metric("#it inner", lambda l, it: l.linear_solver_iterations.sum(), 0),
    num_lin_solve=Metric("#lin-solve", lambda l, it: l._static.solver.num_linear_solves, 0),
    num_jac_eval=Metric("#res-eval", lambda l, it: l._static.solver.num_residual_evaluations, 0),
    num_res_eval=Metric("#jac-eval", lambda l, it: l._static.solver.num_jacobian_evaluations, 0),
    solver_total_time=Metric("t total (s)", lambda l, it: l._static.solver.total_time_in_seconds, 0),
    solver_logging_time=Metric("t logging (s)", lambda l, it: l._static.solver.logging_time_in_seconds, 0),
    solver_preprocessor_time=Metric("t preproc. (s)", lambda l, it: l._static.solver.preprocessor_time_in_seconds, 0),
    solver_minimizer_time=Metric("t minim. (s)", lambda l, it: l._static.solver.minimizer_time_in_seconds, 0),
    solver_postprocessor_time=Metric("t postproc. (s)", lambda l, it: l._static.solver.postprocessor_time_in_seconds, 0),
    solver_linear_solver_time=Metric("t lin-solve (s)", lambda l, it: l._static.solver.linear_solver_time_in_seconds, 0),
    solver_residual_evaluation_time=Metric("t res-eval (s)", lambda l, it: l._static.solver.residual_evaluation_time_in_seconds, 0),
    solver_jacobian_evaluation_time=Metric("t jac-eval (s)", lambda l, it: l._static.solver.jacobian_evaluation_time_in_seconds, 0),
    solver_avg_residual_evaluation_time=Metric("t avg res-eval (s)", lambda l, it: l._static.solver.residual_evaluation_time_in_seconds / l._static.solver.num_residual_evaluations, 0),
    solver_avg_jacobian_evaluation_time=Metric("t avg jac-eval (s)", lambda l, it: l._static.solver.jacobian_evaluation_time_in_seconds/ l._static.solver.num_jacobian_evaluations , 0),
    # FIXME
    minimizer_time_per_it=Metric("minim. time / it (s)", lambda l, it: l._static.solver.minimizer_time_in_seconds / l.iteration[it], 1),
    stage1_time=Metric("stage 1 time (s)", lambda l, it: l.stage1_time.sum(), 1),
    stage2_time=Metric("stage 2 time (s)", lambda l, it: l.stage2_time.sum(), 1),
    stage1_time_per_it=Metric("stage 1 time / it (s)", lambda l, it: l.stage1_time.sum() / l.iteration[it], 1),
    stage2_time_per_it=Metric("stage 2 time / it (s)", lambda l, it: l.stage2_time.sum() / l.iteration[it], 1),
    cg_time=Metric("cg time (s)", lambda l, it: l.solve_reduced_system_time.sum(), 1),
    cg_time_per_inner_it=Metric("cg-time / 1000-inner-it (s)", lambda l, it: ((l.solve_reduced_system_time.sum() if "solve_reduced_system_time" in l else 0)*1000) / l.linear_solver_iterations.sum(), 1),
    solver_minimizer_minus_cg_time=Metric("t minim. - cg (s)", lambda l, it: l._static.solver.minimizer_time_in_seconds - l.solve_reduced_system_time.sum(), 0),
    solver_minimizer_minus_cg_time_per_outer_it=Metric("(t minim. - cg) / outer-it (s)", lambda l, it: (l._static.solver.minimizer_time_in_seconds - l.solve_reduced_system_time.sum()) / (len(l.solve_reduced_system_time)-1), 0),
    lin_solve_time_per_inner_it=Metric("lin-solve-time / inner-it (s)", lambda l, it: get_linear_solver_time(l) / l.linear_solver_iterations.sum(), 0),
    resident_memory_peak=Metric("mem peak (GB)", lambda l, it: l._static.solver.resident_memory_peak / (2**30), 1),
)
# yapf: enable


def metrics_from_config(spec):

    def get_from_spec(m):
        if isinstance(m, str):
            obj = copy.copy(metric_desc[m])
        else:
            obj = copy.copy(metric_desc[m.name])
            obj.set_config(m)
        if obj.relative_to_metric is not None:
            obj.relative_to_metric = get_from_spec(obj.relative_to_metric)

        return obj

    return [get_from_spec(m) for m in spec]
