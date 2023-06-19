#
# BSD 3-Clause License
#
# This file is part of the RootBA project.
# https://github.com/NikolausDemmel/rootba
#
# Copyright (c) 2021-2023, Nikolaus Demmel.
# All rights reserved.
#
import numpy as np
import os
import math
import functools
import re

import matplotlib
from matplotlib import cm

matplotlib.use('Agg')  # Not to use X server. For TravisCI.
import matplotlib.pyplot as plt  # noqa
from matplotlib.ticker import MaxNLocator
from matplotlib.ticker import IndexLocator
from matplotlib.ticker import FixedLocator
from matplotlib.ticker import LogLocator
from matplotlib.ticker import NullFormatter
from matplotlib.ticker import ScalarFormatter

prop_cycle = plt.rcParams['axes.prop_cycle']

#default_cycler = (cycler(linestyle=['-', '--', ':', '-.']) *
#                  cycler(color=prop_cycle.by_key()['color']))


class ModulusList(list):

    def __init__(self, *args, **kwargs):
        list.__init__(self, *args, **kwargs)

    def __getitem__(self, key):
        return list.__getitem__(self, key % len(self))


default_colors = ModulusList(prop_cycle.by_key()['color'])
default_lines = ModulusList(["-", "-", ":", "--", "-.", ":", "--", "-."])
default_markers = ModulusList(["o", "s", "^", "X", "D", "P", "v", "h"])

from collections import deque
from collections import defaultdict

from pylatex import Figure
from pylatex.utils import NoEscape

from .containers import ExperimentsContainer
from .performance_profiles import get_cost_threshold, get_index_to_threshold
from .performance_profiles import get_performance_profile
from .performance_profiles import get_relative_times


class NoFloatFigure(Figure):
    pass


class Plot(ExperimentsContainer):

    def __init__(self, exps, spec, seq_displayname_mapping, export_basepath):
        super().__init__(seq_displayname_mapping)

        self.width = None

        plotters = dict(
            bal_bar=self.plot_bal_bar,
            bal_cost=self.plot_bal_cost,
            performance_profile=self.performance_profile,
            best_solver=self.best_solver,
            memory=self.memory,
            runtime=self.runtime,
            time_per_inner_it=self.time_per_inner_it,
        )

        plot_fn = plotters[spec.type]
        plot_fn(exps, spec)

        if spec.width is not None:
            self.width = spec.width
        elif self.width is None:
            self.width = 1

        plt.tight_layout()

        saved_file = self._save_plot(spec, export_basepath)

        if "sequence" in spec:
            plot_name = '{} {} {}'.format(spec.type, spec.name, spec.sequence).replace("_", " ")
        else:
            plot_name = '{} {}'.format(spec.type, spec.name).replace("_", " ")

        #with self.create(Subsection(spec.name, numbering=False)) as p:
        with self.create(NoFloatFigure()) as f:
            f.add_image(os.path.abspath(saved_file), width=NoEscape(r'{}\textwidth'.format(self.width)))
            f.add_caption(plot_name)

        # cleanup
        plt.close('all')

    def plot_bal_bar(self, exps, spec):

        # TODO: Add a variant that averages runtimes over a set of sequences. Not sure what the best way to do this is,
        # but maybe averaging relative runtime (alpha) with and averaging for the steps averaging fractions of total
        # runtime (both geometric mean).

        num = len(spec.bal_bar_tolerances)

        if spec.figsize is None:
            spec.figsize = [6.4 * num, 4.8]
        fig, axes = plt.subplots(1, num, figsize=spec.figsize)

        if num == 1:
            axes = [axes]

        for i, tol in enumerate(spec.bal_bar_tolerances):
            ax = axes[i]
            self.plot_bal_bar_one(exps, spec, ax, tol)

    def plot_bal_bar_one(self, exps, spec, ax1, tol):

        # extract list of logs and solver names for specified sequence
        logs = [exps[e].runs[spec.sequence].log for e in spec.experiments]
        solver_names = [exps[e].display_name for e in spec.experiments]

        # parse tol and extract list of outer-iteration indices up to which to compute runtimes
        try:

            # if it's a string, try to convert to float first (toml doesn't allow heterogeneous arrays)
            if isinstance(tol, str):
                try:
                    tol = float(tol)
                except ValueError:
                    pass

            if isinstance(tol, float):
                # Interpret tolerance as an accuracy threshold like for performance profiles. Every solver has a
                # different number of outer iterations to reach the specified accuracy or might also be failed.
                idx_to_thresh = get_index_to_threshold(exps, spec.experiments, tol, filter_regex=spec.sequence)
                final_it_idx = np.array([idx_to_thresh[exps[e].name][spec.sequence] for e in spec.experiments])
                title = "tolerance $\\tau$ = {}".format(tol)
            elif isinstance(tol, str) and tol.startswith("it"):
                # Expect tolerance to be string "itX", where integer X specifies how many iterations to include for all
                # solvers. We can also pass -1 to signify: all iterations.
                target_it = int(tol[2:])
                if target_it >= 0:
                    # If a solver converged before the given iteration, just include all iterations.
                    final_it_idx = np.array([min(target_it, len(l.iteration) - 1) for l in logs])
                    title = "after {} iterations".format(target_it)
                elif target_it == -1:
                    # Specifying -1 for iteration count means all iterations for all solvers.
                    final_it_idx = np.array([len(l.iteration) - 1 for l in logs])
                    title = "after all iterations"
                else:
                    raise ValueError
            else:
                raise ValueError()
        except ValueError:
            raise RuntimeError(
                f"Invalid tolerance '{tol}'. Expected float or string 'itX' where X is an integer >= -1.")

        # check for each solver if it succeeded (to reach the accuracy defined by 'tolerance')
        solvers_succeeded = final_it_idx >= 0

        # Mapping of individual runtimes to stages. Note that this for the bal_qr solver. There is also not a 1-to-1
        # correspondence, since for example compute_preconditioner_time is in part in stage 1 or stage 2 and in part
        # later.
        #
        # For now we don't need this. We just check if a solver was executed as "staged" and if not, don't include
        # the runtimes for stage1 and stage2 in the bar plot to show individual step times.
        #
        # stages = {
        #     "stage1_time": [
        #         "jacobian_evaluation_time",
        #         "scale_landmark_jacobian_time",
        #         "compute_preconditioner_time",  # part stage1, part stage2, part after stage 2
        #         "perform_qr_time"
        #     ],
        #     "stage2_time": [
        #         "scale_pose_jacobian_time",
        #         "landmark_damping_time",
        #         "compute_preconditioner_time",  # part stage1, part stage2, part after stage 2
        #         "compute_gradient_time",
        #     ]
        # }

        # maybe_staged_solvers = ["bal_qr"]

        # Determine if run was executed "staged". We check if jacobian_evaluation_time was logged,
        # which should not be the case with staged execution.
        def is_staged(l):
            return np.sum(l.jacobian_evaluation_time) == 0

        # helper to sum per-iteration logged runtimes up to (and including) final_it_idx
        def get_cumulative(step_name):

            def should_be_zero(i, l):
                if i < 0:
                    # failed run: return 0 for all times
                    return True
                if not is_staged(l) and step_name.startswith("stage"):
                    # not-staged run: return 0 for "stage" times else we count those runtimes twice
                    return True
                return False

            return np.array(
                [np.sum(l[step_name][:i + 1]) if not should_be_zero(i, l) else 0 for i, l in zip(final_it_idx, logs)])

        # preprocess time is LMB allocation, etc before LM iterations start
        preprocess = np.array(
            [l._static.solver.preprocessor_time_in_seconds if ok else 0 for ok, l in zip(solvers_succeeded, logs)])

        # sum per-iteration runtimes for different stanges in the LM iteration
        linearize = get_cumulative("jacobian_evaluation_time")
        scale = get_cumulative("scale_pose_jacobian_time") + get_cumulative("scale_landmark_jacobian_time")
        marginalize_qr = get_cumulative("perform_qr_time")
        stage1 = get_cumulative("stage1_time")
        damp_lms = get_cumulative("landmark_damping_time")
        prepare = get_cumulative("prepare_time") + get_cumulative("compute_gradient_time")
        preconditioner = get_cumulative("compute_preconditioner_time")
        stage2 = get_cumulative("stage2_time")
        pcg = get_cumulative("solve_reduced_system_time")
        update = get_cumulative("back_substitution_time") + get_cumulative("update_cameras_time")
        evaluate = get_cumulative("residual_evaluation_time")

        # Get total runtime for comparison with runtime of all parts.
        # Summing iteration times does not take into account preprocessing time -> add it as well.
        total = get_cumulative("iteration_time") + preprocess

        # Get total number of inner iterations and number of inner iterations per each outer iteration. We plot these
        # on a second axis, b/c for many solvers this is the main factor for total runtime.
        num_inner_iterations = get_cumulative("linear_solver_iterations")
        inner_iterations = [l.linear_solver_iterations[:i + 1] if i >= 0 else [] for i, l in zip(final_it_idx, logs)]

        # All steps we want to include in the stacked bar plot and their labels.
        steps = [
            preprocess, linearize, scale, marginalize_qr, stage1, damp_lms, prepare, preconditioner, stage2, pcg,
            update, evaluate
        ]
        step_names = [
            "preprocess", "linearize", "scale", "marginalize_qr", "stage1", "damp_lms", "prepare", "preconditioner",
            "stage2", "PCG", "update", "evaluate"
        ]

        # Prepare xaxis values and width of bar plots.
        major_width = 0.6  # width of main stacked bar plot with runtimes of different steps
        minor_width = 0.15  # width of additional bar plot that indicates number of iterations
        bar_offset = (major_width + minor_width) / 2  # x-axis offset of minor bar from the main one
        ind = np.arange(len(logs))  # x-axis values for center of main bar plot
        colors = reversed(cm.get_cmap("Set3").colors)  # colors of different steps in stacked bar plot

        # plot steps as spaced bar plot
        stacked_values = np.zeros(len(logs))
        for name, step, color in zip(step_names, steps, colors):
            ax1.bar(ind, step, major_width, label=name, color=color, bottom=stacked_values)
            stacked_values += step

        if spec.plot_total_runtime:
            # plot another thin bar with total runtime for comparison with the individual steps (to make sure they
            # account for everything)
            tot_width = minor_width / 2
            ax1.bar(ind - bar_offset + tot_width / 2, total, tot_width, label="total", color="lightgray")

        ax1.set_xticks(ind)
        if spec.xticks_show_iterations:
            # in the xticks that indicate the solver name, show the number of outer iterations that are plotted, or also
            # indicate failure (to reach the given accuracy).
            xticklabels = [
                "{}\n({})".format(name, f"{num_it} it" if num_it >= 0 else "failed")
                for name, num_it in zip(solver_names, final_it_idx)
            ]
        else:
            # just solver names
            xticklabels = solver_names
        ax1.set_xticklabels(xticklabels,
                            rotation=spec.xticks_rotate,
                            horizontalalignment=spec.xticks_horizontalalignment)

        if spec.xticks_color_failed:
            # make failed solver's xticks red
            for i, success in enumerate(solvers_succeeded):
                if not success:
                    ax1.get_xticklabels()[i].set_color('red')

        # axis label and plot title
        ax1.set_ylabel("runtime (accumulated) [s]")
        ax1.set_title(title)

        # ask matplotlib for the plotted objects and their labels --> both axes in one legend
        lines, labels = ax1.get_legend_handles_labels()
        legend_axis = ax1  # if no twin axis, we will later add a legend to ax1

        # add twin axis with number of inner iterations
        if spec.plot_inner_iterations:
            ax2 = ax1.twinx()
            ax2.bar(ind + bar_offset, num_inner_iterations, minor_width, label="iterations", color=default_colors[0])
            ax2.set_ylabel("# inner iterations")

            if spec.plot_inner_iterations_outer_iterations:
                # indicate how inner iterations are related to outer iterations by horizontal lines on top of the bar
                # plot
                for all_iterations, x_center in zip(inner_iterations, ind + bar_offset):
                    xmin = x_center - 0.5 * minor_width
                    xmax = x_center + 0.5 * minor_width
                    ax2.hlines(y=np.cumsum(all_iterations),
                               xmin=xmin,
                               xmax=xmax,
                               linestyle="-",
                               linewidth=0.3,
                               color="lightgray")

            # ask matplotlib for the plotted objects and their labels --> both axes in one legend
            lines2, labels2 = ax2.get_legend_handles_labels()
            lines = lines + lines2
            labels = labels + labels2
            legend_axis = ax2

        # add legend (one legend for both axes)
        legend_axis.legend(lines, labels, loc=spec.legend_loc, fontsize="x-small")

    def plot_bal_cost(self, exps, spec):

        logs = [exps[e].runs[spec.sequence].log for e in spec.experiments]
        names = [exps[e].display_name for e in spec.experiments]

        xmaxit = functools.reduce(max, ((l.iteration.max() if l is not None else -math.inf) for l in logs))
        xmaxt = functools.reduce(max, ((l.cumulative_time.max() if l is not None else -math.inf) for l in logs))
        xmaxit += 0.5
        xminit = -0.5
        xmaxt += 1

        runtimes = [l.cumulative_time[-1] for l in logs if l is not None]
        fastest_time = min(runtimes) if runtimes else None

        num_plots = len(spec.bal_cost_include)
        assert num_plots > 0

        if spec.figsize is None:
            spec.figsize = [5 * num_plots, 3]

        fig, actual_axes = plt.subplots(1, num_plots, figsize=spec.figsize)

        if num_plots == 1:
            actual_axes = deque([actual_axes])
        else:
            actual_axes = deque(actual_axes)
        axes = []
        for variant in ("cost_time", "cost_it", "tr_radius", "inner_it", "memory", "cost_time_without_preprocess"):
            if variant in spec.bal_cost_include:
                axes.append(actual_axes.popleft())
            else:
                axes.append(None)
        (ax1, ax2, ax3, ax4, ax5, ax6) = axes

        tolerance_thresholds = [get_cost_threshold(logs, tol) for tol in spec.tolerances]

        for i, (l, name) in enumerate(zip(logs, names)):
            zorder = i
            if l is not None:
                if spec.reverse_zorder:
                    zorder = len(names) - zorder
                kwargs = dict(label=name, color=default_colors[i], zorder=zorder)
                if ax1 is not None:
                    if spec.plot_cost_semilogy:
                        ax1.semilogy(l.cumulative_time, l.cost, default_lines[i], **kwargs)
                    else:
                        ax1.plot(l.cumulative_time, l.cost, default_lines[i], **kwargs)
                if ax2 is not None:
                    if spec.plot_cost_semilogy:
                        ax2.semilogy(l.iteration, l.cost, default_lines[i], **kwargs)
                    else:
                        ax2.plot(l.iteration, l.cost, default_lines[i], **kwargs)
                if ax3 is not None:
                    ax3.semilogy(l.iteration, l.trust_region_radius, default_lines[i], **kwargs)
                if ax4 is not None:
                    ax4.plot(l.iteration, l.linear_solver_iterations, default_lines[i], **kwargs)
                if ax5 is not None:
                    ax5.semilogy(l.iteration, l.resident_memory_peak / 2**30, default_lines[i], **kwargs)
                if ax6 is not None:
                    cumulative_process_time = l.cumulative_time - l._static.solver.preprocessor_time_in_seconds
                    kwargs["label"] = "{0:.1f}".format(l._static.solver.preprocessor_time_in_seconds) + "s"

                    if spec.plot_cost_semilogy:
                        ax6.semilogy(cumulative_process_time, l.cost, default_lines[i], **kwargs)
                    else:
                        ax6.plot(cumulative_process_time, l.cost, default_lines[i], **kwargs)

        def get_tol_limit(factor):
            diff = math.log10(max(tolerance_thresholds)) - math.log10(min(tolerance_thresholds))
            return 10**(math.log10(min(tolerance_thresholds)) + diff * factor)

        if ax1 is not None:
            if isinstance(spec.ylabel, str):
                ax1.set_ylabel(spec.ylabel)
            elif spec.ylabel:
                ax1.set_ylabel("cost")
            ax1.set_xlabel("time [s]")
            if spec.ylim_tolerance.top is not None and fastest_time is not None:
                ax1.set_ylim(top=get_tol_limit(spec.ylim_tolerance.top))
            if spec.ylim_tolerance.bottom is not None and fastest_time is not None:
                ax1.set_ylim(bottom=get_tol_limit(spec.ylim_tolerance.bottom))
            if spec.ylim_cost.top is not None:
                ax1.set_ylim(top=spec.ylim_cost.top)
            if spec.ylim_cost.bottom is not None:
                ax1.set_ylim(bottom=spec.ylim_cost.bottom)
            if spec.xlim_time_fastest.left is not None and fastest_time is not None:
                ax1.set_xlim(left=spec.xlim_time_fastest.left * fastest_time)
            if spec.xlim_time_fastest.right is not None and fastest_time is not None:
                ax1.set_xlim(right=min(xmaxt, spec.xlim_time_fastest.right * fastest_time))
            if spec.xlim_time.left is not None:
                ax1.set_xlim(left=spec.xlim_time.left)
            if spec.xlim_time.right is not None:
                ax1.set_xlim(right=spec.xlim_time.right)
            ax1.get_yaxis().set_major_locator(LogLocator(base=10, subs="auto"))
            ax1.get_yaxis().set_major_formatter(ScalarFormatter())
            ax1.ticklabel_format(axis='y', style='sci', scilimits=(0, 0), useMathText=True)
            ax1.get_yaxis().set_minor_formatter(NullFormatter())
        if ax2 is not None:
            ax2.set_ylabel("cost")
            ax2.set_xlabel("iteration")
            if spec.ylim_tolerance.top is not None and fastest_time is not None:
                ax2.set_ylim(top=get_tol_limit(spec.ylim_tolerance.top))
            if spec.ylim_tolerance.bottom is not None and fastest_time is not None:
                ax2.set_ylim(bottom=get_tol_limit(spec.ylim_tolerance.bottom))
            if spec.ylim_cost.top is not None:
                ax2.set_ylim(top=spec.ylim_cost.top)
            if spec.ylim_cost.bottom is not None:
                ax2.set_ylim(bottom=spec.ylim_cost.bottom)
            if spec.xlim_it.left is not None:
                ax2.set_xlim(left=spec.xlim_it.left)
            if spec.xlim_it.right is not None:
                ax2.set_xlim(right=spec.xlim_it.right)
            ax2.get_xaxis().set_major_locator(MaxNLocator(6))  #integer=True))

            # TODO: fix this case for when there are no valid logs; and also for when manual xlimit is specified
            # ax2.set_xlim([xminit, xmaxit])

            ax2.get_yaxis().set_major_locator(LogLocator(base=10, subs="auto"))
            ax2.get_yaxis().set_major_formatter(ScalarFormatter())
            ax2.ticklabel_format(axis='y', style='sci', scilimits=(0, 0), useMathText=True)
            ax2.get_yaxis().set_minor_formatter(NullFormatter())
            #ax2.tick_params(axis='y', labelcolor=color)
        if ax3 is not None:
            ax3.set_ylabel("trust region radius")
            ax3.set_xlabel("iteration")
            ax3.get_xaxis().set_major_locator(MaxNLocator(6))  #integer=True))

            # TODO: fix this case for when there are no valid logs; and also for when manual xlimit is specified
            # ax2.set_xlim([xminit, xmaxit])
        if ax4 is not None:
            ax4.set_ylabel("#it linear solver")
            ax4.set_xlabel("iteration")
            ax4.get_xaxis().set_major_locator(MaxNLocator(6))  #integer=True))

            # TODO: fix this case for when there are no valid logs; and also for when manual xlimit is specified
            # ax2.set_xlim([xminit, xmaxit])
        if ax5 is not None:
            ax5.set_ylabel("peak memory [GB]")
            ax5.set_xlabel("iteration")
            ax5.get_xaxis().set_major_locator(MaxNLocator(6))  #integer=True))

            # TODO: fix this case for when there are no valid logs; and also for when manual xlimit is specified
            # ax2.set_xlim([xminit, xmaxit])
            ax5.set_ylim(top=100, bottom=0.01)
        if ax6 is not None:
            if isinstance(spec.ylabel, str):
                ax6.set_ylabel(spec.ylabel)
            elif spec.ylabel:
                ax6.set_ylabel("cost")
            ax6.set_xlabel("time [s]")
            if spec.ylim_tolerance.top is not None and fastest_time is not None:
                ax6.set_ylim(top=get_tol_limit(spec.ylim_tolerance.top))
            if spec.ylim_tolerance.bottom is not None and fastest_time is not None:
                ax6.set_ylim(bottom=get_tol_limit(spec.ylim_tolerance.bottom))
            if spec.ylim_cost.top is not None:
                ax6.set_ylim(top=spec.ylim_cost.top)
            if spec.ylim_cost.bottom is not None:
                ax6.set_ylim(bottom=spec.ylim_cost.bottom)
            if spec.xlim_time_fastest.left is not None and fastest_time is not None:
                ax6.set_xlim(left=spec.xlim_time_fastest.left * fastest_time)
            if spec.xlim_time_fastest.right is not None and fastest_time is not None:
                ax6.set_xlim(right=min(xmaxt, spec.xlim_time_fastest.right * fastest_time))
            if spec.xlim_time.left is not None:
                ax6.set_xlim(left=spec.xlim_time.left)
            if spec.xlim_time.right is not None:
                ax6.set_xlim(right=spec.xlim_time.right)
            ax6.get_yaxis().set_major_locator(LogLocator(base=10, subs="auto"))
            ax6.get_yaxis().set_major_formatter(ScalarFormatter())
            ax6.ticklabel_format(axis='y', style='sci', scilimits=(0, 0), useMathText=True)
            ax6.get_yaxis().set_minor_formatter(NullFormatter())

        if spec.plot_tolerances:
            xmin1, xmax1 = ax1.get_xlim()
            for thresh in tolerance_thresholds:
                if thresh < math.inf:
                    kwargs = dict(linestyles=":", color="dimgray", linewidth=0.75, zorder=0)
                    if ax1 is not None:
                        ax1.hlines(thresh, xmin=xmin1, xmax=xmax1, **kwargs)
                    if ax2 is not None:
                        ax2.hlines(thresh, xmin=xminit, xmax=xmaxit, **kwargs)

        if spec.title:
            for ax in axes:
                if ax:
                    ax.set_title(spec.title)

        if spec.suptitle:
            fig.suptitle(spec.suptitle, y=0.9)

        if ax2:
            ax2.legend(loc=spec.legend_loc)
        elif ax1:
            ax1.legend(loc=spec.legend_loc)
        if ax6:
            ax6.legend(loc=spec.legend_loc)

    def performance_profile(self, exps, spec):

        # TODO:
        # - x-axis iterations or time
        # - which cost? cost, avg cost, mean residual?
        # - log vs linear cost thresholds

        num = len(spec.tolerances)

        if spec.figsize is None:
            spec.figsize = [3.7 * num, 2.8]
        fig, axes = plt.subplots(1, num, figsize=spec.figsize)

        if num == 1:
            axes = [axes]

        for i, tol in enumerate(spec.tolerances):

            ax = axes[i]

            (times, pp) = get_performance_profile(exps,
                                                  spec.experiments,
                                                  tol,
                                                  filter_regex=spec.filter_regex,
                                                  subtract_preprocessor_time=spec.subtract_preprocessor_time)

            for j, (name, fractions) in enumerate(pp.items()):
                zorder = j
                if spec.reverse_zorder:
                    zorder = len(pp) - zorder
                display_name = exps[name].display_name
                ax.plot(times,
                        fractions * 100,
                        default_lines[j],
                        label=display_name,
                        color=default_colors[j],
                        zorder=zorder)
            ax.set_title("tolerance $\\tau$ = {}".format(tol))
            ax.set_ylabel("percentage")
            ax.set_xlabel("relative time $\\alpha$")
            #ax.get_yaxis().set_major_locator(MaxNLocator(integer=True))
            if spec.xlimits.right:
                ax.set_xlim(right=spec.xlimits.right[i])
            # if spec.xlimits.left:
            #     ax.set_xlim(left=spec.xlimits.left[i])
            if i == 1:
                ax.legend(loc=spec.legend_loc)
            ax.set_xlim(left=1)  # hardcode start at 1
            xmin, xmax = ax.get_xlim()
            yticks = [0, 20, 40, 60, 80, 100]
            ax.set_yticks(yticks)
            xpad = 0.01 * (xmax - xmin)
            ypad = 0.025 * (yticks[-1] - yticks[0])
            ax.set_xlim(xmin - xpad, xmax + xpad)
            ax.set_ylim(yticks[0] - ypad, yticks[-1] + ypad)
            for p in yticks:
                ax.plot([xmin - xpad, xmax + xpad], [p, p], ":", color="gainsboro", linewidth=0.25, zorder=0)
            ax.get_xaxis().set_major_locator(FixedLocator([1, 2] + list(range(5, int(xmax) + 1, 5))))
            ax.get_xaxis().set_minor_locator(IndexLocator(base=1, offset=xpad))

    def best_solver(self, exps, spec):
        num_rows = len(spec.tolerances)
        num_cols = len(spec.experiments)

        if spec.figsize is None:
            spec.figsize = [3.7 * num_cols, 2.8 * num_rows]
        fig, axes = plt.subplots(num_rows, num_cols, figsize=spec.figsize)

        rcs_sparsity = dict()
        num_cams = dict()
        for exp in exps.values():
            for s in exp.sequences():
                if hasattr(exp.runs[s].log, "_static"):
                    rcs_sparsity[s] = exp.runs[s].log._static.problem_info.rcs_sparsity
                    num_cams[s] = exp.runs[s].log._static.problem_info.num_cameras

        if num_rows * num_cols == 1:
            axes = [axes]

        # only works with python>=3.7 and plt>=3.4
        fig.supxlabel("#cameras")
        fig.supylabel("RCS sparsity (fraction)")

        # row title
        for ax, tol in zip(axes[:, 0], spec.tolerances):
            ax.set_ylabel("$\\tau$ = {}".format(tol))

        # column title
        for ax, exp in zip(axes[0], spec.experiments):
            ax.set_title(exps[exp].display_name)

        min_rel_time_colormap = 1.0
        max_rel_time_colormap = 3.0

        def get_colormap_value(v):
            if v < math.inf:
                # values < 1 should not occur for relative runtime
                assert v >= min_rel_time_colormap
                # clamp at given maximum
                return min(v, max_rel_time_colormap)
            else:
                return math.inf

        scatterplot = None

        for i, tol in enumerate(spec.tolerances):
            relative_times_by_seq = get_relative_times(exps,
                                                       spec.experiments,
                                                       tol,
                                                       filter_regex=spec.filter_regex,
                                                       subtract_preprocessor_time=spec.subtract_preprocessor_time)
            seq_names = relative_times_by_seq.keys()

            seq_num_cams = np.array([num_cams[s] for s in seq_names])
            seq_hpp_sparsity = np.array([rcs_sparsity[s] for s in seq_names])

            for j, experiment in enumerate(spec.experiments):
                ax = axes[i][j]
                colors = np.array(
                    [get_colormap_value(rel_times[experiment]) for rel_times in relative_times_by_seq.values()])
                is_fastest_mask = colors == 1.0
                is_inf_mask = ~np.isfinite(colors)
                is_other_mask = np.logical_and(~is_fastest_mask, ~is_inf_mask)
                if spec.color_by_relative_runtime:
                    scatterplot = ax.scatter(seq_num_cams[is_other_mask],
                                             seq_hpp_sparsity[is_other_mask],
                                             c=colors[is_other_mask],
                                             cmap=cm.get_cmap("viridis"),
                                             vmin=min_rel_time_colormap,
                                             vmax=max_rel_time_colormap,
                                             s=spec.marker_size)
                else:
                    ax.scatter(seq_num_cams[is_other_mask],
                               seq_hpp_sparsity[is_other_mask],
                               c="gray",
                               s=spec.marker_size)
                ax.scatter(seq_num_cams[is_inf_mask],
                           seq_hpp_sparsity[is_inf_mask],
                           c="fuchsia",
                           marker="x",
                           s=spec.marker_size * 3)
                ax.scatter(seq_num_cams[is_fastest_mask],
                           seq_hpp_sparsity[is_fastest_mask],
                           c="red",
                           marker="d",
                           s=spec.marker_size * 2)
                ax.set_xscale("log")
                ylim_padding = 0.05
                ax.set_ylim(0.0 - ylim_padding, 1.0 + ylim_padding)

        if scatterplot is not None:
            # plot single colormap for all subplots
            fig.colorbar(scatterplot, ax=axes.ravel().tolist())

        if spec.suptitle:
            fig.suptitle(spec.suptitle)

    def memory(self, exps, spec):
        # check peak memory after iteration 1 (first 'real' iteration after the initialization)
        self._plot_vs_problem_size(
            exps, spec, "memory (GB)", lambda l: l.resident_memory_peak[1] / (1024 * 1024 * 1024)
            if len(l.resident_memory_peak) >= 2 else math.nan)

    def runtime(self, exps, spec):
        # check peak memory after iteration 1 (first 'real' iteration after the initialization)
        self._plot_vs_problem_size(exps, spec, "runtime (s)", lambda l: l._static.solver.total_time_in_seconds)

    def time_per_inner_it(self, exps, spec):
        # check peak memory after iteration 1 (first 'real' iteration after the initialization)
        self._plot_vs_problem_size(
            exps, spec, "time / inner_it (s)",
            lambda l: l._static.solver.linear_solver_time_in_seconds / l.linear_solver_iterations.sum())

    def _plot_vs_problem_size(self, exps, spec, label, value_fn):

        num_plots = len(spec.problem_size_variants)
        assert num_plots > 0

        if spec.figsize is None:
            spec.figsize = [4 * num_plots, 3]

        fig, actual_axes = plt.subplots(1, num_plots, figsize=spec.figsize)

        if num_plots == 1:
            actual_axes = deque([actual_axes])
        else:
            actual_axes = deque(actual_axes)
        axes = []
        for variant in ("cam", "lm", "obs"):
            if variant in spec.problem_size_variants:
                axes.append(actual_axes.popleft())
            else:
                axes.append(None)
        (ax1, ax2, ax3) = axes

        for i, exp_name in enumerate(spec.experiments):
            zorder = i
            if spec.reverse_zorder:
                zorder = len(spec.experiments) - zorder

            seq_names = exps[exp_name].sequences(filter_regex=spec.filter_regex)
            seq_names, logs = zip(
                *[(s, exps[exp_name].runs[s].log) for s in seq_names if not exps[exp_name].runs[s].is_failed()])
            display_name = exps[exp_name].display_name

            data = np.array([value_fn(l) for l in logs])
            num_cams = np.array([l._static.problem_info.num_cameras for l in logs])
            num_lms = np.array([l._static.problem_info.num_landmarks for l in logs]) / 1e6
            num_obs = np.array([l._static.problem_info.num_observations for l in logs]) / 1e6

            label_prefix = "{}: ".format(display_name)
            scatter_label = None if spec.best_fit_line else display_name

            plot_args = dict(color=default_colors[i], linewidth=0.25, zorder=zorder)
            plot_args_scatter = dict(label=scatter_label, s=spec.marker_size, marker=default_markers[i])

            if ax1:
                ax1.scatter(num_cams, data, **plot_args_scatter, **plot_args)
                if spec.best_fit_line:
                    self._plot_best_fit_line(ax1, num_cams, data, label_prefix=label_prefix, **plot_args)
                ax1.set_xlabel("#cameras")
                ax1.set_ylabel(label)
                if spec.legend_loc:
                    ax1.legend(loc=spec.legend_loc)

            if ax2:
                ax2.scatter(num_lms, data, **plot_args_scatter, **plot_args)
                if spec.best_fit_line:
                    self._plot_best_fit_line(ax2, num_lms, data, label_prefix=label_prefix, **plot_args)
                ax2.set_xlabel("#landmarks (millions)")
                ax2.set_ylabel(label)
                if spec.legend_loc:
                    ax2.legend(loc=spec.legend_loc)

            if ax3:
                ax3.scatter(num_obs, data, **plot_args_scatter, **plot_args)
                if spec.best_fit_line:
                    self._plot_best_fit_line(ax3, num_obs, data, label_prefix=label_prefix, **plot_args)
                ax3.set_xlabel("#observations (millions)")
                ax3.set_ylabel(label)
                if spec.legend_loc:
                    ax3.legend(loc=spec.legend_loc)

        if spec.suptitle:
            fig.suptitle(spec.suptitle)

    def _plot_best_fit_line(self, ax, x, y, label_prefix="", **kwargs):
        # https://stackoverflow.com/a/31800660/1813258
        line = np.polyfit(x, y, 1)
        label = "{}{:.1e}*x + {:.1e}".format(label_prefix, line[0], line[1])
        ax.plot(np.unique(x), np.poly1d(line)(np.unique(x)), label=label, **kwargs)

    # static:
    filename_counters = defaultdict(int)

    def _save_plot(self, spec, basepath, extension=".pdf"):

        os.makedirs(basepath, exist_ok=True)

        if "sequence" in spec:
            filename = '{}_{}_{}'.format(spec.type, spec.name, spec.sequence)
        else:
            filename = '{}_{}'.format(spec.type, spec.name)

        filename = filename.replace(" ", "_").replace("/", "_")

        Plot.filename_counters[filename] += 1
        counter = Plot.filename_counters[filename]
        if counter > 1:
            filename = "{}-{}".format(filename, counter)

        filepath = os.path.join(basepath, "{}.{}".format(filename, extension.strip('.')))

        plt.savefig(filepath)

        return filepath
