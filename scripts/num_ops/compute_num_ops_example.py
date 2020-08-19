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

n_poses = 1778
n_landmarks = 993923
n_obs = 5001946

num_iter_cg = 100

dim_poses = 9
dim_landmarks = 3
dim_res = 2

N_p = n_poses * dim_poses
# N_l = n_landmarks * dim_landmarks
N_r = n_obs * dim_res

# cost of QR decomposition
n_qr = ops_nm_decomposition(n_landmarks, N_p, N_r)
print('n_qr\t\t', n_qr, '\t\t', n_qr * 1e-12)

# cost of Schur complement computation
n_sc = ops_schur_complement(n_landmarks, dim_poses, N_p, N_r)
print('n_sc\t\t', n_sc, '\t', n_sc * 1e-12)

# # cost of explicit Hessian computation
# print('qr_h\t\t', qr_h, '\t', qr_h * 1e-12)

# cost of CGNR (on QR-reduced system)
lscg_mat_op_all = ops_cgnr(N_r - 3 * n_landmarks, N_p, num_iter_cg)
print('lscg_mat_op_all\t', lscg_mat_op_all, '\t', lscg_mat_op_all * 1e-12)

# total QR cost (decomposition + CGNR)
n_qr_lscg = n_qr + lscg_mat_op_all
print('n_qr_lscg\t', n_qr_lscg, '\t', n_qr_lscg * 1e-12)

# cost of CG (on Schur-reduced system)
cg_mat_op_all = ops_conjugate_gradient(N_p, num_iter_cg)
print('cg_mat_op_all\t', cg_mat_op_all, '\t\t', cg_mat_op_all * 1e-12)

# total Schur cost (SC computation + CG)
n_sc_cg = n_sc + cg_mat_op_all
print('n_sc_cg\t\t', n_sc_cg, '\t', n_sc_cg * 1e-12)
