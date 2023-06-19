#
# BSD 3-Clause License
#
# This file is part of the RootBA project.
# https://github.com/NikolausDemmel/rootba
#
# Copyright (c) 2021-2023, Nikolaus Demmel.
# All rights reserved.
#
import os

from pylatex import Subsection, Tabular, TextColor
from pylatex import MultiColumn, FootnoteText
from pylatex.utils import italic, bold, NoEscape, escape_latex, dumps_list

from .containers import ExperimentsContainer
from .util import format_ratio_percent
from ..util import bal_order


class OverviewTable(ExperimentsContainer):

    def __init__(self, exps, spec, show_values_failed_runs, seq_displayname_mapping, export_basepath):
        super().__init__(seq_displayname_mapping)

        def format_stats(stats):
            return "{:.1f}+-{:.1f}, [{}, {}]".format(stats.mean, stats.stddev, int(stats.min), int(stats.max))

        accessors_rootba = {
            '#cam':
                lambda l: "{:,}".format(l._static.problem_info.num_cameras),
            '#lm':
                lambda l: "{:,}".format(l._static.problem_info.num_landmarks),
            #'#lm-per-cam':
            #    lambda l: "{:,.1f}".format(l._static.problem_info.num_landmarks / l._static.problem_info.num_cameras),
            '#obs-per-cam':
                lambda l: "{:,.1f}".format(l._static.problem_info.num_observations / l._static.problem_info.num_cameras
                                          ),
            '#obs':
                lambda l: "{:,}".format(l._static.problem_info.num_observations),
            '#obs-per-lm':
                lambda l: format_stats(l._static.problem_info.per_lm_obs),
            '#obs-per-lm-mean':
                lambda l: "{:.1f}".format(l._static.problem_info.per_lm_obs.mean),
            '#obs-per-lm-stddev':
                lambda l: "{:.1f}".format(l._static.problem_info.per_lm_obs.stddev),
            '#obs-per-lm-min':
                lambda l: int(l._static.problem_info.per_lm_obs.min),
            '#obs-per-lm-max':
                lambda l: int(l._static.problem_info.per_lm_obs.max),
            'rcs-sparsity':
                lambda l: "{:.0f}%".format(l._static.problem_info.rcs_sparsity * 100),
        }

        seq_names = sorted(set(
            (s for group in spec.columns for s in exps[group.name].sequences(filter_regex=spec.filter_regex))),
                           key=bal_order)
        # TODO: make order configurable

        column_headers = [(group.name, len(group.metrics)) for group in spec.columns]
        columns = [(group.name, m) for group in spec.columns for m in group.metrics]

        num_col = len(columns)

        t = Tabular('l' + '|r' * num_col)

        header_row = ['']
        for group in column_headers:
            header_row.append(MultiColumn(group[1], align='|c', data=group[0]))
        t.add_row(header_row)
        t.add_hline()

        t.add_row([''] + [c[1] for c in columns])
        t.add_hline()

        for seq in seq_names:
            row = [self.seq_displayname(seq)]
            for c in columns:
                if seq not in exps[c[0]].runs:
                    row.append(TextColor("red", '?'))
                else:
                    exp_name, metric = c
                    run = exps[exp_name].runs[seq]
                    accessors = accessors_rootba

                    treat_as_failed = (run.log is None) if show_values_failed_runs else run.is_failed()

                    if treat_as_failed:
                        row.append(TextColor("red", run.failure_str()))
                    else:
                        value = accessors[metric](run.log)
                        # if same metric is shown in first group, show as percentage
                        if exp_name != spec.columns[0].name and metric in spec.columns[0].metrics:
                            exp0 = exps[spec.columns[0].name]
                            if seq in exp0.runs and not exp0.runs[seq].is_failed():
                                value_group0 = accessors[metric](exp0.runs[seq].log)
                                value = format_ratio_percent(value, value_group0)
                        if run.is_failed():
                            value = TextColor("red", value)
                        row.append(value)

            t.add_row(row)

        if spec.export_latex:
            os.makedirs(export_basepath, exist_ok=True)
            t.generate_tex(os.path.join(export_basepath, spec.export_latex))

        with self.create(Subsection(spec.name, numbering=False)) as p:
            p.append(t)
