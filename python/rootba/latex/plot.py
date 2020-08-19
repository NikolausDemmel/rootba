#
# BSD 3-Clause License
#
# This file is part of the RootBA project.
# https://github.com/NikolausDemmel/rootba
#
# Copyright (c) 2021, Nikolaus Demmel.
# All rights reserved.
#
import numpy as np
import os
import math
import functools

import matplotlib

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
from .performance_profiles import get_cost_threshold
from .performance_profiles import get_performance_profile


class NoFloatFigure(Figure):
    pass


class Plot(ExperimentsContainer):

    def __init__(self, exps, spec, seq_displayname_mapping, export_basepath):
        super().__init__(seq_displayname_mapping)

        self.width = None

        plotters = dict(
            bal_cost=self.plot_bal_cost,
            performance_profile=self.performance_profile,
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
        for variant in ("cost_time", "cost_it", "tr_radius", "inner_it", "memory"):
            if variant in spec.bal_cost_include:
                axes.append(actual_axes.popleft())
            else:
                axes.append(None)
        (ax1, ax2, ax3, ax4, ax5) = axes

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

            (times, pp) = get_performance_profile(exps, spec.experiments, tol, filter_regex=spec.filter_regex)

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
