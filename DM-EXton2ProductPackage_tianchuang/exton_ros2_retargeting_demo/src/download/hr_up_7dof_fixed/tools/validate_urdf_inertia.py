#!/usr/bin/env python3
"""Check inertia alignment against this package's original SolidWorks CSV.

This validates coordinate conversion, not the accuracy of the CAD mass data.
Run with system Python (requires numpy), optionally passing a URDF path.
"""

import argparse
import csv
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def rotation(rpy):
    roll, pitch, yaw = map(float, rpy.split())
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def check_inertial(link, row):
    name = link.get('name')
    visual = link.find('visual/origin')
    inertial = link.find('inertial')
    origin = inertial.find('origin')
    # All original mesh and inertial frames in the supplied CSV have zero RPY;
    # reject other exports instead of silently applying an incorrect conversion.
    for prefix in ('Visual', 'Center of Mass'):
        assert all(float(row[f'{prefix} {axis}']) == 0 for axis in ('Roll', 'Pitch', 'Yaw'))
    assert all(float(row[f'Visual {axis}']) == 0 for axis in 'XYZ')
    transform = rotation(visual.get('rpy'))
    expected_com = transform @ np.array([float(row[f'Center of Mass {a}']) for a in 'XYZ'])
    expected_com += np.fromstring(visual.get('xyz'), sep=' ')
    com = np.fromstring(origin.get('xyz'), sep=' ')
    assert np.allclose(com, expected_com, atol=1e-12, rtol=0), f'{name}: COM misaligned'
    assert np.allclose(rotation(origin.get('rpy')), transform, atol=1e-12, rtol=0), f'{name}: inertia frame misaligned'
    assert math.isclose(float(inertial.find('mass').get('value')), float(row['Mass']), rel_tol=1e-12)
    tensor = inertial.find('inertia')
    for attr, label in [('ixx', 'Ixx'), ('ixy', 'Ixy'), ('ixz', 'Ixz'),
                        ('iyy', 'Iyy'), ('iyz', 'Iyz'), ('izz', 'Izz')]:
        assert math.isclose(float(tensor.get(attr)), float(row['Moment ' + label]), rel_tol=1e-12), f'{name}: tensor value changed'
    matrix = np.array([
        [float(tensor.get('ixx')), float(tensor.get('ixy')), float(tensor.get('ixz'))],
        [float(tensor.get('ixy')), float(tensor.get('iyy')), float(tensor.get('iyz'))],
        [float(tensor.get('ixz')), float(tensor.get('iyz')), float(tensor.get('izz'))],
    ])
    eigenvalues = np.linalg.eigvalsh(matrix)
    assert eigenvalues[0] > 0 and eigenvalues[2] <= eigenvalues[:2].sum() + 1e-12, f'{name}: invalid inertia'
    return com


def validate(urdf_path):
    root = ET.parse(urdf_path).getroot()
    with urdf_path.with_suffix('.csv').open(newline='') as stream:
        rows = {row['Link Name']: row for row in csv.DictReader(stream)}
    count = 0
    for link in root.findall('link'):
        if link.find('inertial') is None:
            continue
        com = check_inertial(link, rows[link.get('name')])
        mesh = link.find('visual/geometry/mesh')
        mesh_path = urdf_path.parent.parent / 'meshes' / mesh.get('filename').rsplit('/', 1)[1]
        data = mesh_path.read_bytes()
        triangles = int.from_bytes(data[80:84], 'little')
        assert len(data) == 84 + 50 * triangles, f'{mesh_path}: invalid binary STL'
        dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
        points = np.frombuffer(data, dtype=dtype, offset=84)['vertices'].reshape(-1, 3).astype(float)
        visual = link.find('visual/origin')
        points = points @ rotation(visual.get('rpy')).T + np.fromstring(visual.get('xyz'), sep=' ')
        assert np.all(com >= points.min(0) - 1e-6) and np.all(com <= points.max(0) + 1e-6), f"{link.get('name')}: COM outside mesh bounds"
        count += 1
    print(f'PASS: {urdf_path} ({count} inertial links: CSV transforms, mass/tensor preservation, positive inertia, COM bounds)')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('urdf', nargs='?', type=Path, default=Path(__file__).resolve().parents[1] / 'src/hr_up_7dof_fixed/urdf/HR-UP-7DOF-3KG-R-B01382-URDF-1029.urdf')
    validate(parser.parse_args().urdf)
