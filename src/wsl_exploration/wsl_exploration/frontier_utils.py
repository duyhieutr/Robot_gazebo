# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
"""
Frontier detection on a 2D occupancy grid. Pure numpy, no ROS imports, so it
can be unit-tested without a ROS installation.

A frontier cell is a known-free cell that touches an unknown cell. Neighbouring
frontier cells form a cluster; the robot drives to a cluster to see what is
behind it, which pushes the frontier outwards until the space is closed.

Grid convention (same as nav_msgs/OccupancyGrid): array[row, col] with
row = y, col = x; value -1 unknown, 0..100 occupancy probability. World
position of the centre of cell (row, col) is
(origin_x + (col + 0.5) * resolution, origin_y + (row + 0.5) * resolution).
"""

from collections import deque, namedtuple
import math

import numpy as np

# cells: (N, 2) int array of (row, col); x, y: world position of the target
# cell (a free cell inside the cluster); length: cluster extent in metres.
Frontier = namedtuple('Frontier', ['x', 'y', 'size', 'length', 'cells'])


def box_any(mask, radius):
    """True where `mask` is True anywhere within `radius` cells (square window)."""
    if radius <= 0:
        return mask.copy()
    height, width = mask.shape
    padded = np.pad(mask.astype(np.int32), radius)
    integral = np.zeros((padded.shape[0] + 1, padded.shape[1] + 1), dtype=np.int32)
    integral[1:, 1:] = padded.cumsum(axis=0).cumsum(axis=1)
    k = 2 * radius + 1
    window = (integral[k:k + height, k:k + width] - integral[0:height, k:k + width]
              - integral[k:k + height, 0:width] + integral[0:height, 0:width])
    return window > 0


def frontier_mask(grid, free_max=25, occupied_min=65, clearance_cells=0):
    """Boolean mask of frontier cells.

    Free cells (0..free_max) with a 4-connected unknown neighbour, minus those
    within `clearance_cells` of an occupied cell (>= occupied_min): the robot
    could not stand there anyway.
    """
    free = (grid >= 0) & (grid <= free_max)
    unknown = grid < 0
    near_unknown = np.zeros_like(unknown)
    near_unknown[1:, :] |= unknown[:-1, :]
    near_unknown[:-1, :] |= unknown[1:, :]
    near_unknown[:, 1:] |= unknown[:, :-1]
    near_unknown[:, :-1] |= unknown[:, 1:]
    mask = free & near_unknown
    if clearance_cells > 0:
        mask &= ~box_any(grid >= occupied_min, clearance_cells)
    return mask


def cluster_cells(mask, min_size=1):
    """Group True cells of `mask` into 8-connected clusters of >= min_size cells."""
    rows, cols = np.nonzero(mask)
    remaining = set(zip(rows.tolist(), cols.tolist()))
    clusters = []
    while remaining:
        seed = remaining.pop()
        queue = deque([seed])
        members = [seed]
        while queue:
            row, col = queue.popleft()
            for d_row in (-1, 0, 1):
                for d_col in (-1, 0, 1):
                    neighbour = (row + d_row, col + d_col)
                    if neighbour in remaining:
                        remaining.remove(neighbour)
                        queue.append(neighbour)
                        members.append(neighbour)
        if len(members) >= min_size:
            clusters.append(np.array(members, dtype=np.int64))
    return clusters


def cell_to_world(row, col, resolution, origin_x, origin_y):
    return (origin_x + (col + 0.5) * resolution, origin_y + (row + 0.5) * resolution)


def find_frontiers(grid, resolution, origin_x, origin_y, free_max=25, occupied_min=65,
                   clearance_m=0.4, min_length_m=0.5):
    """All frontier clusters of the grid, as a list of Frontier.

    The target of a cluster is the member cell closest to the cluster's mean:
    it is free by construction and has the requested clearance, unlike the
    mean itself, which can fall outside a curved frontier.
    """
    clearance_cells = int(math.ceil(clearance_m / resolution))
    min_size = max(1, int(math.ceil(min_length_m / resolution)))
    mask = frontier_mask(grid, free_max, occupied_min, clearance_cells)
    frontiers = []
    for cells in cluster_cells(mask, min_size):
        mean = cells.mean(axis=0)
        target = cells[np.argmin(((cells - mean) ** 2).sum(axis=1))]
        x, y = cell_to_world(target[0], target[1], resolution, origin_x, origin_y)
        extent = (cells.max(axis=0) - cells.min(axis=0) + 1) * resolution
        frontiers.append(Frontier(x, y, len(cells), float(np.hypot(*extent)), cells))
    return frontiers


def pick_frontier(frontiers, robot_x, robot_y, blacklist=(), blacklist_radius=0.6,
                  min_distance=0.5, length_weight=0.5):
    """Best frontier to visit next, or None.

    Cost = distance to the robot - length_weight * frontier length: nearby
    frontiers win, long ones (more unexplored space behind them) are preferred
    over short ones at a similar distance. Frontiers near a blacklisted goal
    (already visited or unreachable) and ones closer than `min_distance` (the
    robot is already there, so going on would not reveal more) are skipped.
    """
    best, best_cost = None, math.inf
    for frontier in frontiers:
        distance = math.hypot(frontier.x - robot_x, frontier.y - robot_y)
        if distance < min_distance:
            continue
        if any(math.hypot(frontier.x - bx, frontier.y - by) < blacklist_radius
               for bx, by in blacklist):
            continue
        cost = distance - length_weight * frontier.length
        if cost < best_cost:
            best, best_cost = frontier, cost
    return best


def still_frontier(frontiers, x, y, radius, resolution, origin_x, origin_y):
    """True if any frontier cell of any cluster lies within `radius` of (x, y).

    Checks every cell, not only each cluster's target: the target shifts while
    a cluster grows, which must not make the current goal look explored.
    """
    for frontier in frontiers:
        cell_x = origin_x + (frontier.cells[:, 1] + 0.5) * resolution
        cell_y = origin_y + (frontier.cells[:, 0] + 0.5) * resolution
        if np.any((cell_x - x) ** 2 + (cell_y - y) ** 2 < radius ** 2):
            return True
    return False
