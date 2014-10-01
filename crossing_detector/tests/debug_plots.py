#!/usr/bin/python
# -*- coding: utf-8 -*-

# File formats (space separated):
# /tmp/{node_name}_place_profile.dat: x y
# /tmp/{node_name}_delaunay_input.dat: x y
# /tmp/{node_name}_delaunay_edges.dat: x1 y1 x2 y2
# /tmp/{node_name}_delaunay_circumcircles.dat: x y r

from __future__ import print_function, division

import os
import sys
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


def get_filename(node, dataname):
    filename = '/tmp/' + node + '_' + dataname + '.dat'
    if not os.path.exists(filename):
        raise ValueError('File {} does not exist'.format(filename))
    return filename


def load_data(node, dataname):
    try:
        return np.loadtxt(get_filename(node, dataname))
    except ValueError, e:
        print(e.message)
        return None


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


def do_plots(node):
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1, aspect='equal')

    x = np.array([1, -1, -1, 1, 1]) * map_width / 2
    y = np.array([1, 1, -1, -1, 1]) * map_height / 2
    ax.plot(x, y, color='k', alpha=0.5)

    if do_plot_place_profile:
        place_profile = load_data(node, 'place_profile')
        if place_profile is not None:
            plt.plot(place_profile[:, 0], place_profile[:, 1],
                     'r.', markersize=4,
                     label='place_profile')

    if do_plot_delaunay_input:
        delaunay_input = load_data(node, 'delaunay_input')
        if delaunay_input is not None:
            plt.plot(delaunay_input[:, 0], delaunay_input[:, 1],
                     'b.', markersize=4,
                     label='delaunay_input')

    if do_plot_delaunay_edges:
        edges = load_data(node, 'delaunay_edges')
        if edges is not None:
            plot_edges(ax, edges, color='b', label='delaunay_edges')

    if do_plot_candidates:
        candidates = load_data(node, 'candidates')
        if candidates is not None:
            plot_candidates(ax, candidates, color=[0, 0.8, 0], alpha=0.1,
                            label='candidates')

    if do_plot_rejected:
        rejected = load_data(node, 'rejected')
        if rejected is not None:
            plot_candidates(ax, rejected, color=[0.9, 0, 0], alpha=0.1,
                            label='rejected')

    if (len(ax.lines) > 1) or ax.collections:
        ax.legend()
        ax.grid(True)

        plt.show()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: {} node_name'.format(sys.argv[0]) +
              ' [place_profile]' +
              ' [delaunay_input]' +
              ' [delaunay_edges]' +
              ' [candidates]' +
              ' [rejected]')
        exit()
    if len(sys.argv) > 2:
        do_plot_place_profile = 'place_profile' in sys.argv
        do_plot_delaunay_input = 'delaunay_input' in sys.argv
        do_plot_delaunay_edges = 'delaunay_edges' in sys.argv
        do_plot_candidates = 'candidates' in sys.argv
        do_plot_rejected = 'rejected' in sys.argv
    else:
        do_plot_place_profile = True
        do_plot_delaunay_input = True
        do_plot_delaunay_edges = False
        do_plot_candidates = False
        do_plot_rejected = False
    do_plots(sys.argv[1])
