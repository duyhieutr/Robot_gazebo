# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
"""Unit tests for the frontier detection: pure numpy, no ROS needed."""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from wsl_exploration import frontier_utils as fu  # noqa: E402

RES = 0.05


def make_room(size=100):
    """Known free square with an occupied wall ring, surrounded by unknown."""
    grid = np.full((size + 20, size + 20), -1, dtype=np.int8)
    grid[10:10 + size, 10:10 + size] = 0
    grid[10, 10:10 + size] = 100
    grid[9 + size, 10:10 + size] = 100
    grid[10:10 + size, 10] = 100
    grid[10:10 + size, 9 + size] = 100
    return grid


def test_closed_room_has_no_frontier():
    assert fu.find_frontiers(make_room(), RES, 0.0, 0.0) == []


def test_gap_in_wall_is_a_frontier():
    grid = make_room()
    grid[10, 50:70] = 0  # 1 m opening in the bottom wall, free cells touch unknown
    grid[9, 50:70] = -1
    frontiers = fu.find_frontiers(grid, RES, 0.0, 0.0, clearance_m=0.0)
    assert len(frontiers) == 1
    assert 0.9 < frontiers[0].length < 1.3
    assert grid[int(round(frontiers[0].y / RES - 0.5)),
                int(round(frontiers[0].x / RES - 0.5))] == 0


def test_cells_next_to_walls_are_dropped_by_clearance():
    grid = make_room()
    grid[10, 50:70] = 0
    grid[9, 50:70] = -1
    # 0.4 m clearance: only cells >= 8 cells from any wall remain; the opening
    # lies in the wall line, so nothing is left.
    assert fu.find_frontiers(grid, RES, 0.0, 0.0, clearance_m=0.4) == []


def test_open_boundary_of_explored_area_is_a_frontier():
    grid = np.full((120, 120), -1, dtype=np.int8)
    grid[40:80, 40:80] = 0  # free patch in the middle of unknown space
    frontiers = fu.find_frontiers(grid, RES, -3.0, -3.0, clearance_m=0.0, min_length_m=0.2)
    assert len(frontiers) == 1  # one ring around the patch
    assert frontiers[0].size == 4 * 40 - 4


def test_tiny_clusters_are_filtered():
    grid = np.full((60, 60), -1, dtype=np.int8)
    grid[30, 30] = 0  # a single speckle
    assert fu.find_frontiers(grid, RES, 0.0, 0.0, clearance_m=0.0, min_length_m=0.5) == []


def test_box_any_matches_brute_force():
    rng = np.random.default_rng(0)
    mask = rng.random((30, 40)) > 0.97
    got = fu.box_any(mask, 3)
    expected = np.zeros_like(mask)
    for row, col in zip(*np.nonzero(mask)):
        expected[max(0, row - 3):row + 4, max(0, col - 3):col + 4] = True
    assert np.array_equal(got, expected)


def test_world_conversion_uses_cell_centres():
    assert fu.cell_to_world(0, 0, 0.05, -1.0, 2.0) == (-1.0 + 0.025, 2.0 + 0.025)


def test_pick_prefers_near_and_long_and_skips_blacklist():
    near = fu.Frontier(1.0, 0.0, 10, 0.5, np.zeros((1, 2), dtype=int))
    far = fu.Frontier(6.0, 0.0, 10, 0.5, np.zeros((1, 2), dtype=int))
    long_far = fu.Frontier(4.0, 0.0, 80, 4.0, np.zeros((1, 2), dtype=int))
    assert fu.pick_frontier([near, far], 0.0, 0.0) is near
    assert fu.pick_frontier([near, long_far], 0.0, 0.0, length_weight=1.0) is long_far
    assert fu.pick_frontier([near, far], 0.0, 0.0, blacklist=[(1.0, 0.1)]) is far
    assert fu.pick_frontier([near], 0.9, 0.0, min_distance=0.5) is None
    assert fu.pick_frontier([], 0.0, 0.0) is None


def test_still_frontier_checks_every_cell():
    cells = np.array([[10, 10], [10, 11], [10, 12]])
    frontier = fu.Frontier(0.0, 0.0, 3, 0.15, cells)
    x = 0.0 + 12.5 * RES
    y = 0.0 + 10.5 * RES
    assert fu.still_frontier([frontier], x, y, 0.1, RES, 0.0, 0.0)
    assert not fu.still_frontier([frontier], x + 2.0, y, 0.1, RES, 0.0, 0.0)
    assert not fu.still_frontier([], x, y, 0.1, RES, 0.0, 0.0)
