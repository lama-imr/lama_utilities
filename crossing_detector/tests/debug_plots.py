#!/usr/bin/python
# -*- coding: utf-8 -*-

# File formats (space separated):
# /tmp/place_profile.dat: x y
# /tmp/delaunay_input.dat: x y
# /tmp/delaunay_edges.dat: x1 y1 x2 y2
# /tmp/delaunay_circumcircles.dat: x y r

from __future__ import print_function, division

import numpy as np
import matplotlib
matplotlib.use('Qt4Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.collections import PatchCollection
from matplotlib.patches import Circle

# Map size in meters.
map_width = 4
map_height = 4


def plot_edges(ax, edges, **kwargs):
    segs = []
    for x1, y1, x2, y2 in edges:
        segs.append([[x1, y1], [x2, y2]])
    col = LineCollection(segs, **kwargs)
    ax.add_collection(col)


def plot_candidates(ax, candidates, **kwargs):
    rmax = candidates[:, 2].max()
    patches = []
    for x, y, r in candidates:
        patches.append(Circle((x, y), r))
        if r == rmax:
            # Add the best candidate twice, wo that it will be darker
            # if transparency is used
            patches.append(Circle((x, y), r))
    col = PatchCollection(patches, **kwargs)
    ax.add_collection(col)


def do_plots():
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1, aspect='equal')

    x = np.array([1, -1, -1, 1, 1]) * map_width / 2
    y = np.array([1, 1, -1, -1, 1]) * map_height / 2
    ax.plot(x, y, color='k', alpha=0.5)

    if do_plot_place_profile:
        place_profile = np.loadtxt('/tmp/place_profile.dat')
        plt.plot(place_profile[:, 0], place_profile[:, 1], 'r.',
                 label='place_profile')

    if do_plot_delaunay_input:
        delaunay_input = np.loadtxt('/tmp/delaunay_input.dat')
        plt.plot(delaunay_input[:, 0], delaunay_input[:, 1], 'b.',
                 label='delaunay_input')

    if do_plot_delaunay_edges:
        edges = np.loadtxt('/tmp/delaunay_edges.dat')
        plot_edges(ax, edges, color='b', label='delaunay_edges')

    if do_plot_filt_edges:
        filt_edges = np.loadtxt('filt_edges.dat')
        plot_edges(ax, filt_edges, color='g', linewidth=1.5, label='filt_edges')

    if do_plot_candidates:
        candidates = np.loadtxt('candidates.dat')
        plot_candidates(ax, candidates, color=[0, 0, 0.9], alpha=0.1,
                        label='candidates')

    ax.legend()
    ax.grid(True)

    plt.show()

if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        do_plot_place_profile = 'place_profile' in sys.argv
        do_plot_delaunay_input = 'delaunay_input' in sys.argv
        do_plot_delaunay_edges = 'delaunay_edges' in sys.argv
        do_plot_filt_edges = 'filt_edges' in sys.argv
        do_plot_candidates = 'candidates' in sys.argv
    else:
        do_plot_place_profile = True
        do_plot_delaunay_input = True
        do_plot_delaunay_edges = False
        do_plot_filt_edges = False
        do_plot_candidates = False
    do_plots()
