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


def ops_nm_decomposition(n_l, N_p, N_r):
    "Compute number of flops for nullspace marginalization (excluding linear system solution)"
    N_qr = 72 * (N_r - 2 * n_l)  # QR decomposition of block-sparse landmark Jacobian
    N_qj = 18 * (N_r - 2 * n_l) * (N_p + 1)  # multiplication of Q' with (Jp|r)
    N_bs = 6 * n_l * (N_p + 2)  # back substitution step
    return N_qr + N_qj + N_bs


def ops_schur_complement(n_l, d_p, N_p, N_r):
    "Compute number of flops for setting up Schur-reduced system and back-substitution"
    N_bl = 2 * N_r * (d_p * d_p + 4 * d_p + 12)  # compute sub-blocks of Hessian and RHS vector
    N_red = 6 * n_l * (N_p * N_p + 4 * N_p + 7)  # compute reduced system
    N_bs = 6 * n_l * (N_p + 3)  # back substitution step
    return N_bl + N_red + N_bs


def ops_conjugate_gradient(n_r, n_i):
    "Compute number of flops for performing CG on a linear system"
    N_init = 2 * n_r  # initialization (only once)
    N_loop = 2 * n_r * n_r + 8 * n_r + 1  # loop (in each step)
    return N_init + n_i * N_loop


def ops_cgnr(n_r, n_c, n_i):
    N_init = 4 * n_r * n_c + 2 * n_c  # initialization (only once)
    N_loop = 4 * n_r * n_c + 4 * n_r + 6 * n_c + 1  # loop (in each step)
    return N_init + n_i * N_loop
