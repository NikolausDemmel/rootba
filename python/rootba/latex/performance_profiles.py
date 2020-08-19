#
# BSD 3-Clause License
#
# This file is part of the RootBA project.
# https://github.com/NikolausDemmel/rootba
#
# Copyright (c) 2021, Nikolaus Demmel.
# All rights reserved.
#
import warnings
import math
import numpy as np

from collections import OrderedDict
from collections import defaultdict
from munch import Munch


def get_cost_threshold(logs, tolerance):
    if not any(logs):
        return math.inf
    initial_cost = max(l.cost[0] for l in logs if l is not None)
    best_cost = min(min(l.cost) for l in logs if l is not None)
    cost_thresh = best_cost + tolerance * (initial_cost - best_cost)
    return cost_thresh


def get_performance_profile(exps, names, tolerance, filter_regex=None):

    es = OrderedDict()
    seq_names = set()
    exp_by_seq = defaultdict(set)

    for i, exp_name in enumerate(names):
        exp = exps[exp_name]
        e = Munch()
        e.name = exp_name
        e.display_name = exp.display_name
        e.logs = {s: exp.runs[s].log for s in exp.sequences(filter_regex=filter_regex) if not exp.runs[s].is_failed()}
        seq_names.update(e.logs.keys())
        for s in e.logs:
            exp_by_seq[s].add(exp_name)
        es[exp_name] = e

    # compute initial costs for each sequence
    initial_costs = {s: max(es[name].logs[s].cost[0] for name in exp_names) for s, exp_names in exp_by_seq.items()}

    # sanity check of initial costs
    initial_costs_all = {s: [es[name].logs[s].cost[0] for name in exp_names] for s, exp_names in exp_by_seq.items()}
    initial_costs_min = {s: min(es[name].logs[s].cost[0] for name in exp_names) for s, exp_names in exp_by_seq.items()}
    for s in seq_names:
        ratio = (initial_costs[s] - initial_costs_min[s]) / initial_costs[s]
        if ratio > 1e-2:
            warnings.warn("initial costs not equal for {}. max: {}, min: {}, ratio: {}; {}".format(
                s, initial_costs[s], initial_costs_min[s], ratio, initial_costs_all[s]))

    # compute best costs and cost threshold (according to tolerance) for each sequence
    best_costs = {s: min(min(es[name].logs[s].cost) for name in exp_names) for s, exp_names in exp_by_seq.items()}
    cost_thresh = {s: best_costs[s] + tolerance * (initial_costs[s] - best_costs[s]) for s in seq_names}

    # for each solver and each sequence, determine the iteration index when the solver first achieved a value below the threshold
    idx_to_thresh = {}
    for name, e in es.items():
        indices = {}
        for s in e.logs.keys():
            below_thresh = e.logs[s].cost <= cost_thresh[s]
            if any(below_thresh):
                indices[s] = np.argmax(e.logs[s].cost <= cost_thresh[s])
            else:
                indices[s] = -1
        idx_to_thresh[name] = indices

    # for each solver and each sequence, determine the time to reach the threshold
    time_to_thresh = {}
    for name, e in es.items():
        indices = idx_to_thresh[name]
        times = {}
        for s, idx in indices.items():
            if idx >= 0:
                times[s] = e.logs[s].cumulative_time[idx]
            else:
                times[s] = math.inf
        time_to_thresh[name] = times

    # for each sequence, compute the minimum time to reach the threshold
    min_times = {s: min(time_to_thresh[name][s] for name in names) for s, names in exp_by_seq.items()}

    # TODO: remove hack
    #for s in ["venice1778"]:  #, "ladybug138"]:
    #    if s in seq_names:
    #        print("min time {}, tol {}: {}".format(s, tolerance, min_times[s]))
    #        for name, times in time_to_thresh.items():
    #            print("{}: {}".format(name, 1 / (times[s] / min_times[s])))

    # list all unqiue relative times where something changes (x axis of performance profile)
    all_relative_times = np.array(
        sorted(set(
            t / min_times[s] for name, times in time_to_thresh.items() for s, t in times.items() if t < math.inf)))

    # for each solver, compute performance profile value at every time step
    performances = {}
    for name, times in time_to_thresh.items():
        relative_times = sorted((t / min_times[s] for s, t in times.items() if t < math.inf))
        counter = 0
        perf = np.zeros((len(all_relative_times),))
        for i, t in enumerate(all_relative_times):
            while counter < len(relative_times) and relative_times[counter] <= all_relative_times[i]:
                counter += 1
            perf[i] = counter / len(seq_names)
        performances[name] = perf

    return (all_relative_times, performances)
