# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
"""
Turn a 2D occupancy grid into a 3D mesh by extruding occupied cells upwards,
and write meshes and coloured point clouds to disk. Pure numpy, no ROS imports.

The height is a parameter, not a measurement: the 2D map has no vertical
information. Use it for a clean building-like model; use the point cloud for
what the sensors actually saw.

Grid convention as in frontier_utils: array[row, col], row = y, col = x,
value -1 unknown, 0..100 occupancy. A mesh is an (N, 3) float array of
vertices, three consecutive vertices per triangle, counter-clockwise seen
from outside.
"""

import numpy as np

from wsl_exploration import frontier_utils as fu

SIDE_SHADE = 0.55
TOP_SHADE = 0.85


def occupied_mask(grid, occupied_min=65):
    return grid >= occupied_min


def clean_mask(mask, min_cells=8, smooth_cells=1):
    """Close one-cell gaps, then drop connected groups smaller than `min_cells`.

    LiDAR speckle leaves isolated cells and ragged edges; both would turn into
    many tiny boxes. Closing = dilate then erode with a square window.
    """
    if smooth_cells > 0:
        dilated = fu.box_any(mask, smooth_cells)
        mask = ~fu.box_any(~dilated, smooth_cells)
    if min_cells > 1:
        keep = np.zeros_like(mask)
        for cells in fu.cluster_cells(mask, min_cells):
            keep[cells[:, 0], cells[:, 1]] = True
        mask = keep
    return mask


def merge_rectangles(mask):
    """Cover the True cells with few axis-aligned rectangles.

    Returns a list of (row0, row1, col0, col1), half-open. Runs of True cells
    in each row that repeat with the same column span in the next row are
    stacked into one rectangle.
    """
    rectangles = []
    open_runs = {}  # (col0, col1) -> first row
    height = mask.shape[0]
    for row in range(height + 1):
        runs = set()
        if row < height:
            line = np.concatenate(([0], mask[row].astype(np.int8), [0]))
            edges = np.flatnonzero(np.diff(line))
            runs = {(int(a), int(b)) for a, b in zip(edges[0::2], edges[1::2])}
        for span in list(open_runs):
            if span not in runs:
                rectangles.append((open_runs.pop(span), row, span[0], span[1]))
        for span in runs:
            open_runs.setdefault(span, row)
    return rectangles


def box_triangles(x0, y0, x1, y1, z0, z1):
    """Top and four side faces of a box as 10 triangles (no bottom face).

    Returns (vertices (30, 3), shades (30,)): shade is a brightness factor
    per vertex, brighter on top.
    """
    top = [(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    south = [(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)]
    north = [(x1, y1, z0), (x0, y1, z0), (x0, y1, z1), (x1, y1, z1)]
    west = [(x0, y1, z0), (x0, y0, z0), (x0, y0, z1), (x0, y1, z1)]
    east = [(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)]
    vertices, shades = [], []
    for quad, shade in ((top, TOP_SHADE), (south, SIDE_SHADE), (north, SIDE_SHADE),
                        (west, SIDE_SHADE), (east, SIDE_SHADE)):
        for a, b, c in ((0, 1, 2), (0, 2, 3)):
            vertices += [quad[a], quad[b], quad[c]]
            shades += [shade] * 3
    return np.array(vertices, dtype=np.float64), np.array(shades, dtype=np.float64)


def extrude(grid, resolution, origin_x, origin_y, height=2.4, occupied_min=65,
            min_cells=8, smooth_cells=1):
    """Mesh of the occupied cells of `grid` extruded from z = 0 to `height`.

    Returns (vertices (3 * triangles, 3), shades (3 * triangles,)); both empty
    arrays when nothing is occupied.
    """
    mask = clean_mask(occupied_mask(grid, occupied_min), min_cells, smooth_cells)
    parts, shade_parts = [], []
    for row0, row1, col0, col1 in merge_rectangles(mask):
        vertices, shades = box_triangles(
            origin_x + col0 * resolution, origin_y + row0 * resolution,
            origin_x + col1 * resolution, origin_y + row1 * resolution, 0.0, height)
        parts.append(vertices)
        shade_parts.append(shades)
    if not parts:
        return np.zeros((0, 3)), np.zeros(0)
    return np.concatenate(parts), np.concatenate(shade_parts)


def floor_quad(grid, resolution, origin_x, origin_y):
    """Two triangles covering the bounding box of all known cells, at z = 0."""
    rows, cols = np.nonzero(grid >= 0)
    if len(rows) == 0:
        return np.zeros((0, 3))
    x0 = origin_x + cols.min() * resolution
    x1 = origin_x + (cols.max() + 1) * resolution
    y0 = origin_y + rows.min() * resolution
    y1 = origin_y + (rows.max() + 1) * resolution
    return np.array([(x0, y0, 0), (x1, y0, 0), (x1, y1, 0),
                     (x0, y0, 0), (x1, y1, 0), (x0, y1, 0)], dtype=np.float64)


def write_obj(path, vertices):
    """Wavefront OBJ of a triangle soup. Opens in Blender, MeshLab, CloudCompare."""
    count = len(vertices)
    assert count % 3 == 0, 'a triangle mesh needs a multiple of 3 vertices'
    with open(path, 'w') as out:
        out.write('# wsl extruded occupancy map\n')
        for x, y, z in vertices:
            out.write(f'v {x:.4f} {y:.4f} {z:.4f}\n')
        for i in range(1, count + 1, 3):
            out.write(f'f {i} {i + 1} {i + 2}\n')


def decode_xyz_rgb(data, point_step, fields, count):
    """(N, 3) float32 xyz and (N, 3) uint8 rgb from sensor_msgs/PointCloud2 data.

    `fields` maps name -> byte offset. The rgb field is a float32 whose bytes
    are 0x00RRGGBB (PCL convention). Without an rgb field the colours are grey.
    """
    raw = np.frombuffer(bytes(data), dtype=np.uint8).reshape(count, point_step)

    def column(name):
        offset = fields[name]
        return np.ascontiguousarray(raw[:, offset:offset + 4]).view(np.float32).reshape(count)

    xyz = np.stack([column('x'), column('y'), column('z')], axis=1)
    if 'rgb' in fields:
        packed = column('rgb').view(np.uint32)
        rgb = np.stack([(packed >> 16) & 255, (packed >> 8) & 255, packed & 255],
                       axis=1).astype(np.uint8)
    else:
        rgb = np.full((count, 3), 160, dtype=np.uint8)
    finite = np.isfinite(xyz).all(axis=1)
    return xyz[finite], rgb[finite]


def write_ply(path, xyz, rgb):
    """Binary little-endian PLY of a coloured point cloud."""
    assert len(xyz) == len(rgb)
    vertex = np.empty(len(xyz), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                                       ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')])
    vertex['x'], vertex['y'], vertex['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    vertex['red'], vertex['green'], vertex['blue'] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    header = ('ply\nformat binary_little_endian 1.0\n'
              f'element vertex {len(xyz)}\n'
              'property float x\nproperty float y\nproperty float z\n'
              'property uchar red\nproperty uchar green\nproperty uchar blue\n'
              'end_header\n')
    with open(path, 'wb') as out:
        out.write(header.encode('ascii'))
        out.write(vertex.tobytes())
