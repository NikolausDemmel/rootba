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

n_poses = 1778
n_lanmarks = 993923
n_obs = 5001946

num_iter_cg = 100

dim_poses = 9
dim_landmarks = 3
dim_res = 2

n_p = n_poses * dim_poses
n_l = n_lanmarks * dim_landmarks
n_r = n_obs * dim_res

# QR marginalization
num_givens_rotations = 3 * n_r - 6 * n_lanmarks

compute_givens_flops = 6

J_p_row_len = n_p
res_row_len = 1
J_l_row_len = dim_landmarks
all_row_len = J_p_row_len + res_row_len + J_l_row_len

num_flops_per_row_element = 6

givens_operations = all_row_len * num_flops_per_row_element + compute_givens_flops

back_subs = 9 * n_lanmarks + n_l + 2 * n_p * n_l

n_qr = num_givens_rotations * givens_operations + back_subs

print('n_qr\t\t', n_qr, '\t\t', n_qr * 1e-12)

# Schur marginalization

form_hessian = 144 * n_r
inv_H_ll = 42 * n_lanmarks
H_pl_inv_H_ll_mult = n_lanmarks * 2 * n_p * dim_landmarks * dim_landmarks
H_pl_inv_H_ll_H_lp_mult = 2 * n_p * n_p * n_l
H_pl_inv_H_ll_b_l_mult = 2 * n_p * n_l

back_subs_sc = n_lanmarks * 2 * dim_landmarks * dim_landmarks + 2 * n_l * n_p

n_sc = form_hessian + inv_H_ll + H_pl_inv_H_ll_mult + H_pl_inv_H_ll_H_lp_mult + H_pl_inv_H_ll_b_l_mult + back_subs_sc

print('n_sc\t\t', n_sc, '\t', n_sc * 1e-12)

# Form H explicitly for QR

qr_h = 2 * n_p * n_p * (n_r - n_l)
print('qr_h\t\t', qr_h, '\t', qr_h * 1e-12)

# LSCG for QR

lscg_mat_op_per_iter = 2 * 2 * n_p * (n_r - n_l)
lscg_mat_op_all = lscg_mat_op_per_iter * num_iter_cg

n_qr_lscg = n_qr + lscg_mat_op_all
print('lscg_mat_op_all\t', lscg_mat_op_all, '\t', lscg_mat_op_all * 1e-12)
print('n_qr_lscg\t', n_qr_lscg, '\t', n_qr_lscg * 1e-12)

# CG for Schur complement

cg_mat_op_per_iter = 2 * n_p * n_p
cg_mat_op_all = cg_mat_op_per_iter * num_iter_cg

n_sc_cg = n_sc + cg_mat_op_all
print('cg_mat_op_all\t', cg_mat_op_all, '\t\t', cg_mat_op_all * 1e-12)
print('n_sc_cg\t\t', n_sc_cg, '\t', n_sc_cg * 1e-12)
