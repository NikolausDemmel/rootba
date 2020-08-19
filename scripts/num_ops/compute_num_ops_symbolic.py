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

from sympy import *
from operation_counts import *

n_landmarks = symbols('n_l')
num_iter_cg = symbols('n_i')
dim_poses = symbols('d_p')
N_p, N_r = symbols('N_p N_r')

# cost of QR decomposition
n_qr = ops_nm_decomposition(n_landmarks, N_p, N_r)
print('n_qr\t\t', expand(n_qr))

# cost of Schur complement computation
n_sc = ops_schur_complement(n_landmarks, dim_poses, N_p, N_r)
print('n_sc\t\t', expand(n_sc))

# # # cost of explicit Hessian computation
# # print('qr_h\t\t', qr_h, '\t', qr_h * 1e-12)

# cost of CGNR (on QR-reduced system)
lscg_mat_op_all = ops_cgnr(N_r - 3 * n_landmarks, N_p, num_iter_cg)
print('lscg_mat_op_all\t', expand(lscg_mat_op_all))

# total QR cost (decomposition + CGNR)
n_qr_lscg = n_qr + lscg_mat_op_all
print('n_qr_lscg\t', expand(n_qr_lscg))

# cost of CG (on Schur-reduced system)
cg_mat_op_all = ops_conjugate_gradient(N_p, num_iter_cg)
print('cg_mat_op_all\t', expand(cg_mat_op_all))

# total Schur cost (SC computation + CG)
n_sc_cg = n_sc + cg_mat_op_all
print('n_sc_cg\t\t', expand(n_sc_cg))
