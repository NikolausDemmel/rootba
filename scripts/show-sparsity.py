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

import matplotlib.pyplot as plt
import numpy as np

num_res = 150
num_poses = 12
num_landmarks = 25

obs = set()

while len(obs) < num_res:
    rand_pose = np.random.randint(0, high=num_poses)
    rand_lm = np.random.randint(0, high=num_landmarks)
    obs.add((rand_pose, rand_lm))

obs_sorted = sorted(obs, key=lambda x: (x[1], x[0]))

#print(np.array(obs_sorted))

obs_map = {}

for o in obs_sorted:
    if o[1] not in obs_map:
        obs_map[o[1]] = []
    obs_map[o[1]].append(o[0])

#print(obs_map)

Jp = np.ones((num_res, num_poses, 3))
Jl = np.ones((num_res, num_landmarks, 3))

i = 0
for lm_id in sorted(obs_map.keys()):
    poses = obs_map[lm_id]

    for p1 in poses:
        for p2 in poses:
            if p1 == p2:
                Jp[i, p1, :] = np.array([86, 193, 255]) / 255.
            else:
                Jp[i, p2] = np.array([213, 213, 213]) / 255.
        Jl[i, lm_id] = np.array([86, 193, 255]) / 255.
        i += 1

fig, axs = plt.subplots(1, 2)
ax1 = axs[0]
ax2 = axs[1]


def disable_ticks(ax1):
    for xlabel_i in ax1.axes.get_xticklabels():
        xlabel_i.set_visible(False)
        xlabel_i.set_fontsize(0.0)
    for xlabel_i in ax1.axes.get_yticklabels():
        xlabel_i.set_fontsize(0.0)
        xlabel_i.set_visible(False)
    for tick in ax1.axes.get_xticklines():
        tick.set_visible(False)
    for tick in ax1.axes.get_yticklines():
        tick.set_visible(False)


disable_ticks(ax1)
disable_ticks(ax2)

ax1.imshow(Jp, aspect=.15)
ax2.imshow(Jl, aspect=.15 * num_landmarks / num_poses)

plt.savefig('jacobian_sparsity.pdf', format='pdf', dpi=300)

plt.show()
