#
# BSD 3-Clause License
#
# This file is part of the RootBA project.
# https://github.com/NikolausDemmel/rootba
#
# Copyright (c) 2021, Nikolaus Demmel.
# All rights reserved.
#
import argparse
import sys
import os
import fnmatch

#import matplotlib as mpl
#mpl.use('Agg')
import matplotlib.pyplot as plt

from cycler import cycler

from .log import BaLog
from .experiments import load_experiments_config
from .experiments import Experiment

#from itertools import cycle
prop_cycle = plt.rcParams['axes.prop_cycle']
#colors = cycle(prop_cycle.by_key()['color'])

#mpl.rcParams['ps.useafm'] = True
#mpl.rcParams['pdf.use14corefonts'] = True
#mpl.rcParams['text.usetex'] = True
#mpl.rcParams['font.size'] = 15

default_cycler = (cycler(linestyle=['-', '--', ':', '-.']) * cycler(color=prop_cycle.by_key()['color']))

default_colors = prop_cycle.by_key()['color']


def do_plot(logs, args):

    fig = plt.figure(num=None, figsize=(25, 15), dpi=100)

    ((ax1, ax2, ax3), (ax4, ax5, ax6)) = fig.subplots(2, 3)

    for i, (name, props) in enumerate(zip(sorted(logs.keys(), key=lambda a: (logs[a]._type, a)), default_cycler)):
        l = logs[name]
        ax1.semilogy(l.iteration, l.cost, label=name, **props)
        ax1.set_xlabel("iterations")
        ax1.set_ylabel("cost")
        ax2.semilogy(l.cumulative_time, l.cost, label=name, **props)
        ax2.set_xlabel("time")
        ax2.set_ylabel("cost")
        ax3.plot(l.iteration, l.cumulative_time, **props)
        ax3.set_xlabel("iterations")
        ax3.set_ylabel("time (s)")

        ax4.semilogy(l.iteration, l.linear_solver_iterations, **props)
        ax4.set_xlabel("iterations")
        ax4.set_ylabel("num cg iterations")
        ax5.semilogy(l.iteration, l.trust_region_radius, **props)
        ax5.set_xlabel("iterations")
        ax5.set_ylabel("tr radius")
        ax6.plot(l.iteration, l.resident_memory_peak / 2**20, **props)
        ax6.set_xlabel("iterations")
        ax6.set_ylabel("peak mem (MB)")

    if not args.no_legend:
        ax1.legend()


def extract_logs_from_experiment(exp, results_accu):

    def add_one(log, label):
        base_label = label
        i = 1
        while label in results_accu:
            i += 1
            label = "{}-{}".format(base_label, i)
        results_accu[label] = log
        #print("loaded {}".format(label))

    for seq_name, run in exp.runs.items():
        if run.log:
            add_one(run.log, "{}-{}".format(exp.name, seq_name))


def load_all_in(path, results_accu, sequences=None):

    def load_one(filepath, basename):
        label = basename
        if sequences:
            if not any(fnmatch.fnmatch(basename, "*{}*".format(s)) for s in sequences):
                # if basename doesn't match any of the specified sequences, skip it
                return
        i = 1
        while label in results_accu:
            i += 1
            label = "{}-{}".format(basename, i)
        results_accu[label] = BaLog(filepath)
        print("loaded {}".format(label))

    if os.path.isdir(path):
        for (dirpath, dirnames, filenames) in os.walk(path):
            for f in filenames:
                # prefer ubjson
                if f.endswith(".json"):
                    if os.path.exists(os.path.join(dirpath, f.replace(".json", ".ubjson"))):
                        continue

                if f in ["ba_log.ubjson", "ba_log.json"]:
                    fullpath = os.path.join(dirpath, f)
                    basename = os.path.basename(os.path.abspath(dirpath))
                    load_one(fullpath, basename)
    elif os.path.isfile(path):
        folder, f = os.path.split(os.path.abspath(path))
        if f in ["ba_log.ubjson", "ba_log.json"]:
            basename = os.path.basename(folder)
            load_one(path, basename)
    else:
        print("No log found for path {}".format(path))


def plot_log(args):

    if args.experiment_config:
        # config argument given; load runs from experiments config

        # load experiments.toml
        config = load_experiments_config(args.config, args)

        # if requested, list expriments and exit
        if args.list_experiment_names:
            for spec in sorted(config.experiments, key=lambda s: s.name):
                print(spec.name)
            return

        # turn list of sequences (using shell-style (fnmatch) patterns) to regex to pass to get_log_dirs
        filter_regex = None
        if args.sequences:
            filter_regex = "|".join(["({})".format(fnmatch.translate(s)) for s in args.sequences])

        # interpret paths as patterns to match experiment names
        added_exps = set()
        logs = dict()
        for p in args.path:
            matched = []
            for spec in config.experiments:
                if fnmatch.fnmatch(spec.name, p):
                    matched.append(spec)
            if not matched:
                print("Didn't find any experiment for '{}'".format(p))
                sys.exit(1)
            else:
                for spec in sorted(matched, key=lambda x: x.name):
                    if spec.name not in added_exps:
                        exp = Experiment.load_spec(spec,
                                                   config.options.base_path,
                                                   cache_dir=None,
                                                   extra_filter_regex=filter_regex,
                                                   other_specs=config.experiments)
                        extract_logs_from_experiment(exp, logs)
                        added_exps.add(spec.name)
    else:
        # no experiments config, just search path for log files
        paths = args.path
        logs = dict()
        for path in paths:
            load_all_in(path, logs, sequences=args.sequences)

    if not logs:
        print("Nothing found")
        return

    do_plot(logs, args)
    figs = [(plt.gcf(), '')]

    plt.tight_layout()

    if args.save:
        save_dir = os.path.dirname(args.save)
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

        for fig, name in figs:

            filename = args.save + ("_" + name if name != "" else "")

            fig.savefig(filename + ".png", bbox_inches='tight')
            #fig.savefig(filename + ".pdf", bbox_inches='tight')

        with open(args.save + "_command.sh", "w") as f:
            f.writelines([" ".join(sys.argv)])

    if not args.no_gui:
        plt.show()


def main():
    parser = argparse.ArgumentParser("Load multiple BA logs and plot combined results for comparison.")
    parser.add_argument("path", nargs='*', help="base path to look for ba_log.[ub]json files in")
    parser.add_argument("--save", default="", help="save plots (prefix path)")
    parser.add_argument("--no-gui", action="store_true", help="don't show plots (e.g. only save)")
    parser.add_argument("--no-legend", action="store_true", help="don't show legend")
    parser.add_argument(
        "-s",
        "--sequences",
        default=None,
        help=
        "limit to results found for given sequences ('space separated list'); this acts as a filter on the folder name where the log file is found, so it is not limited to filtering only by sequence;"
    )
    parser.add_argument("-e",
                        "--experiment-config",
                        action="store_true",
                        help="If set, interpret the positional arguments as experiment names " +
                        "from an experiments.toml config file")
    parser.add_argument("--list-experiment-names",
                        action="store_true",
                        help="if 'experiments-config' is set, list all existing experiments and exit")
    parser.add_argument("--config", default="experiments.toml", help="specs for experiments to load")
    parser.add_argument("--base-path", default=None, help="overwrite basepath for loading logs defined in the config")

    args = parser.parse_args()

    if args.sequences is not None:
        args.sequences = [s for s in args.sequences.split(" ") if s != ""]

    plot_log(args)


if __name__ == "__main__":
    main()
