#!/usr/bin/env python3
#
# BSD 3-Clause License
#
# This file is part of the RootBA project.
# https://github.com/NikolausDemmel/rootba
#
# Copyright (c) 2021, Nikolaus Demmel.
# All rights reserved.
#

from operation_counts import *

# here, import data!

num_iter_cg = 100

dim_poses = 9
dim_landmarks = 3
dim_res = 2

file_numbers = "bal_numbers.csv"

# LaTeX table headline
print("\\toprule")
print(" & $n_p$ & $n_l$ & $n_o$ & SC + CG & QR + CGNR \\\\")
print("\\midrule")

# loop over datasets
lines = open(file_numbers).readlines()
for line in lines:
    # error / empty line handling
    line2 = line.strip()
    if not line2:
        continue
    data = line2.split(';')
    dataset = data[0]
    n_poses = int(data[1])
    n_landmarks = int(data[2])
    n_obs = int(data[3])

    N_p = n_poses * dim_poses
    # N_l = n_landmarks * dim_landmarks
    N_r = n_obs * dim_res

    # cost of QR decomposition
    n_qr = ops_nm_decomposition(n_landmarks, N_p, N_r)
    # cost of Schur complement computation
    n_sc = ops_schur_complement(n_landmarks, dim_poses, N_p, N_r)
    # cost of CGNR (on QR-reduced system)
    lscg_mat_op_all = ops_cgnr(N_r - 3 * n_landmarks, N_p, num_iter_cg)
    # total QR cost (decomposition + CGNR)
    n_qr_lscg = n_qr + lscg_mat_op_all
    # cost of CG (on Schur-reduced system)
    cg_mat_op_all = ops_conjugate_gradient(N_p, num_iter_cg)
    # total Schur cost (SC computation + CG)
    n_sc_cg = n_sc + cg_mat_op_all

    # fill LaTeX table
    # syntax for scientific notation: "{:.2e}".format(...)
    # syntax for commas as 1000 separator: "{:,}".format(...)
    print(dataset, " & ", \
           "{:,}".format(n_poses), " & ", \
           "{:,}".format(n_landmarks), " & ", \
           "{:,}".format(n_obs), " & ", \
           "{:.2e}".format(n_sc_cg), " & ", \
           "{:.2e}".format(n_qr_lscg), " \\\\")

print("\\bottomrule")
