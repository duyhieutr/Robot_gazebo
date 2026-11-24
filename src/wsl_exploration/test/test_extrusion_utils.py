# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
"""Unit tests for the map extrusion and the cloud writers: pure numpy, no ROS."""

import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from wsl_exploration import extrusion_utils as eu  # noqa: E402

RES = 0.05


def blocks_grid():
    grid = np.full((60, 80), -1, dtype=np.int8)
    grid[5:55, 5:75] = 0
    grid[10:14, 10:50] = 100      # a 40 x 4 cell rack
    grid[30:34, 10:50] = 100      # a second one
    grid[45, 60] = 100            # a single speckle
    return grid


def test_merge_rectangles_covers_exactly_the_mask():
    rng = np.random.default_rng(1)
    mask = rng.random((25, 30)) > 0.55
    rebuilt = np.zeros_like(mask)
    for row0, row1, col0, col1 in eu.merge_rectangles(mask):
        assert not rebuilt[row0:row1, col0:col1].any()   # no overlap
        rebuilt[row0:row1, col0:col1] = True
    assert np.array_equal(rebuilt, mask)


def test_merge_rectangles_stacks_identical_rows():
    mask = np.zeros((10, 10), dtype=bool)
    mask[2:6, 3:8] = True
    assert eu.merge_rectangles(mask) == [(2, 6, 3, 8)]


def test_clean_mask_drops_speckle_and_keeps_racks():
    mask = eu.clean_mask(eu.occupied_mask(blocks_grid()), min_cells=8, smooth_cells=1)
    assert not mask[45, 60]
    assert mask[11, 20] and mask[31, 20]


def test_clean_mask_closes_one_cell_gaps():
    mask = np.zeros((20, 20), dtype=bool)
    mask[5:15, 5:9] = True
    mask[5:15, 10:14] = True      # one-cell gap in between
    closed = eu.clean_mask(mask, min_cells=1, smooth_cells=1)
    assert closed[8, 9]


def test_extrude_height_and_footprint():
    vertices, shades = eu.extrude(blocks_grid(), RES, -2.0, -1.5, height=2.4)
    assert len(vertices) % 3 == 0 and len(vertices) == len(shades)
    assert np.isclose(vertices[:, 2].max(), 2.4) and np.isclose(vertices[:, 2].min(), 0.0)
    # two racks; the speckle is gone. Rack 1 spans x -1.5..0.5, y -1.0..-0.8.
    assert np.isclose(vertices[:, 0].min(), -2.0 + 10 * RES)
    assert np.isclose(vertices[:, 0].max(), -2.0 + 50 * RES)
    assert len(vertices) == 2 * 30   # two boxes of 10 triangles each


def test_box_faces_point_outwards():
    vertices, _ = eu.box_triangles(0, 0, 2, 1, 0, 3)
    centre = np.array([1.0, 0.5, 1.5])
    for i in range(0, len(vertices), 3):
        a, b, c = vertices[i:i + 3]
        normal = np.cross(b - a, c - a)
        assert normal @ ((a + b + c) / 3 - centre) > 0, i


def test_empty_grid_gives_empty_mesh():
    vertices, shades = eu.extrude(np.full((10, 10), -1, dtype=np.int8), RES, 0, 0)
    assert vertices.shape == (0, 3) and shades.shape == (0,)


def test_floor_covers_known_cells():
    quad = eu.floor_quad(blocks_grid(), RES, -2.0, -1.5)
    assert len(quad) == 6
    assert np.isclose(quad[:, 0].min(), -2.0 + 5 * RES)
    assert np.isclose(quad[:, 0].max(), -2.0 + 75 * RES)


def test_obj_roundtrip(tmp_path):
    vertices, _ = eu.extrude(blocks_grid(), RES, 0.0, 0.0)
    path = tmp_path / 'm.obj'
    eu.write_obj(str(path), vertices)
    lines = path.read_text().splitlines()
    assert sum(line.startswith('v ') for line in lines) == len(vertices)
    faces = [line for line in lines if line.startswith('f ')]
    assert len(faces) == len(vertices) // 3
    assert max(int(i) for line in faces for i in line.split()[1:]) == len(vertices)


def make_cloud(points, colours, step=20):
    """PointCloud2 data as bytes: x y z at 0, 4, 8, rgb at 16."""
    data = bytearray()
    for (x, y, z), (r, g, b) in zip(points, colours):
        packed = (r << 16) | (g << 8) | b
        data += struct.pack('<fff', x, y, z) + b'\x00' * 4
        data += struct.pack('<I', packed) + b'\x00' * (step - 20)
    return bytes(data)


def test_decode_xyz_rgb_and_skip_nan():
    points = [(1.0, 2.0, 3.0), (float('nan'), 0.0, 0.0), (-1.5, 0.25, 2.0)]
    colours = [(255, 0, 0), (1, 2, 3), (10, 20, 30)]
    data = make_cloud(points, colours, step=24)
    xyz, rgb = eu.decode_xyz_rgb(data, 24, {'x': 0, 'y': 4, 'z': 8, 'rgb': 16}, 3)
    assert xyz.tolist() == [[1.0, 2.0, 3.0], [-1.5, 0.25, 2.0]]
    assert rgb.tolist() == [[255, 0, 0], [10, 20, 30]]


def test_decode_without_rgb_is_grey():
    data = struct.pack('<fff', 1, 2, 3)
    xyz, rgb = eu.decode_xyz_rgb(data, 12, {'x': 0, 'y': 4, 'z': 8}, 1)
    assert xyz.tolist() == [[1.0, 2.0, 3.0]] and rgb.tolist() == [[160, 160, 160]]


def test_ply_header_and_size(tmp_path):
    xyz = np.array([[0, 0, 0], [1, 2, 3]], dtype=np.float32)
    rgb = np.array([[255, 0, 0], [0, 255, 0]], dtype=np.uint8)
    path = tmp_path / 'c.ply'
    eu.write_ply(str(path), xyz, rgb)
    raw = path.read_bytes()
    header, body = raw.split(b'end_header\n', 1)
    assert b'element vertex 2' in header and b'binary_little_endian' in header
    assert len(body) == 2 * (3 * 4 + 3)
    assert struct.unpack('<fff', body[15:27]) == (1.0, 2.0, 3.0)
    assert list(body[27:30]) == [0, 255, 0]
