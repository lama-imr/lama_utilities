#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function, division

import numpy as np
import matplotlib.pyplot as plt

g_filename = '/tmp/goal_theta_velocity_scans.txt'
g_vfactor_scale = 2
g_robot_radius = 0.25
g_safety_distance = 0.1
g_safety_distance_quick = 0.5
g_lim_plot = 5


def show_one_snd_result(num, goalx, goaly, theta_traj, vfactor, scan):
    fig = plt.figure(num, dpi=300)
    ax = fig.add_subplot(1, 1, 1, aspect='equal')
    ax.set_xlim([-g_lim_plot, g_lim_plot])
    ax.set_ylim([-g_lim_plot, g_lim_plot])
    circle_safety = plt.Circle((0, 0), g_robot_radius + g_safety_distance,
                               color='r', alpha=0.5)
    ax.add_artist(circle_safety)
    circle_safety_quick = plt.Circle((0, 0), g_safety_distance_quick,
                                     color='orange', alpha=0.3)
    ax.add_artist(circle_safety_quick)
    theta = np.linspace(-np.pi, np.pi, len(scan), endpoint=False)
    ax.plot(scan * np.cos(theta), scan * np.sin(theta), '.')
    ax.arrow(0, 0, goalx, goaly,
             head_width=0.05, head_length=0.1, fc='k', ec='k')
    ax.arrow(0, 0, vfactor * g_vfactor_scale * np.cos(theta_traj),
             vfactor * g_vfactor_scale * np.sin(theta_traj),
             head_width=0.1, head_length=0.1, fc='r', ec='r')
    ax.grid(True)
    fig.savefig('/tmp/snd{:02d}.png'.format(num))
    plt.close(fig)

goal_theta_vel_scans = np.loadtxt(g_filename)

for i, goal_theta_vel_scan in enumerate(goal_theta_vel_scans):
    goalx = goal_theta_vel_scan[0]
    goaly = goal_theta_vel_scan[1]
    theta_traj = goal_theta_vel_scan[2]
    vfactor = goal_theta_vel_scan[3]
    scan = goal_theta_vel_scan[4:]
    show_one_snd_result(i, goalx, goaly, theta_traj, vfactor, scan)
