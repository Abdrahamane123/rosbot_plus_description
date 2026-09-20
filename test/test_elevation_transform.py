"""Unit-test the elevation map's transform chain against known geometry.

Written after five rounds of chasing a discrepancy through the full simulation
stack - sim, LIO, mapper, detector, probe - where every run took twenty minutes
and carried four independent sources of noise. The symptom was that a plateau
0.11 m above the ground, well covered on both sides, came out of the map at the
SAME height as the ground.

A synthetic scene with exactly known geometry removes every one of those
variables. If the map cannot place a plateau it was handed directly, no amount
of tuning upstream would have helped.
"""

import math

import numpy as np
import pytest
import rclpy

from rosbot_plus_description.elevation_map import ElevationMap


GROUND_Z = 0.0
PLATEAU_Z = 0.11
PLATEAU_X = 3.0
SENSOR_Z_IN_BASE = 0.349          # livox_joint height, from rosbot_plus.xacro


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def make_node(**params):
    overrides = [rclpy.parameter.Parameter(k, value=v) for k, v in params.items()]
    return ElevationMap.__new__(ElevationMap), overrides


def build_node(tilt_deg=0.0, robot_x=0.0):
    """An ElevationMap wired up by hand, with the transforms it would get
    from TF and odometry, so no ROS graph is needed."""
    node = ElevationMap.__new__(ElevationMap)
    # Minimal state the mapping code touches, mirroring __init__.
    node.res = 0.05
    node.length = 12.0
    node.n = int(round(node.length / node.res))
    node.map_frame = 'world'
    node.register = True
    node.reported = True
    node.center = np.array([robot_x, 0.0])
    node.have_center = True
    node.max_z = np.full((node.n, node.n), np.nan, dtype=np.float32)
    node.min_z = np.full((node.n, node.n), np.nan, dtype=np.float32)
    node.count = np.zeros((node.n, node.n), dtype=np.float32)

    # sensor -> base_footprint: a pure translation up, plus the mount tilt.
    t = math.radians(tilt_deg)
    node._R_sb = np.array([[math.cos(t), 0, math.sin(t)],
                           [0, 1, 0],
                           [-math.sin(t), 0, math.cos(t)]])
    node._t_sb = np.array([0.265, 0.0, SENSOR_Z_IN_BASE])
    # base_footprint -> world: robot on flat ground, no rotation.
    node.pose = (np.eye(3), np.array([robot_x, 0.0, 0.0]))
    return node


def sensor_points_for(world_pts, node):
    """Invert the chain: what the sensor must have measured for these world
    points to be correct. Guarantees the test exercises the real transform
    rather than a re-derivation of it."""
    R_wb, t_wb = node.pose
    base = (np.asarray(world_pts) - t_wb) @ R_wb
    return (base - node._t_sb) @ node._R_sb


@pytest.mark.parametrize('tilt', [0.0, 20.0, 25.0])
def test_world_points_round_trip(tilt):
    """A point placed in the world must come back at the same place."""
    node = build_node(tilt_deg=tilt, robot_x=0.5)
    world = np.array([
        [2.0, 0.0, GROUND_Z],
        [3.5, 0.0, PLATEAU_Z],
        [4.0, 1.0, PLATEAU_Z],
        [1.0, -0.5, GROUND_Z],
    ])
    sensor = sensor_points_for(world, node)
    back = node._to_world(sensor, 'livox_frame')
    assert back is not None
    np.testing.assert_allclose(back, world, atol=1e-9)


def test_map_reports_the_step_height():
    """The whole point: a ground plane and a plateau handed to the mapper must
    come out separated by exactly the step height."""
    node = build_node(tilt_deg=25.0, robot_x=0.5)

    rng = np.random.default_rng(0)
    n = 4000
    x = rng.uniform(1.0, 5.0, n)
    y = rng.uniform(-1.0, 1.0, n)
    z = np.where(x < PLATEAU_X, GROUND_Z, PLATEAU_Z)
    world = np.stack([x, y, z], axis=1)

    pts = node._to_world(sensor_points_for(world, node), 'livox_frame')

    # Same binning the node does in _on_cloud.
    rel = pts[:, :2] - node.center
    idx = np.floor(rel / node.res).astype(int) + node.n // 2
    inside = ((idx[:, 0] >= 0) & (idx[:, 0] < node.n)
              & (idx[:, 1] >= 0) & (idx[:, 1] < node.n))
    idx, zz = idx[inside], pts[inside, 2]
    flat = idx[:, 0] * node.n + idx[:, 1]
    cur = node.max_z.reshape(-1)
    fresh = np.isnan(cur[flat])
    cur[np.unique(flat[fresh])] = -np.inf
    np.maximum.at(cur, flat, zz)

    xs = node.center[0] + (np.arange(node.n) - node.n // 2 + 0.5) * node.res
    ys = node.center[1] + (np.arange(node.n) - node.n // 2 + 0.5) * node.res
    corridor = np.abs(ys) < 0.9
    near = (xs > PLATEAU_X - 1.5) & (xs < PLATEAU_X - 0.2)
    far = (xs > PLATEAU_X + 0.3) & (xs < PLATEAU_X + 1.5)

    zn = node.max_z[np.ix_(near, corridor)]
    zf = node.max_z[np.ix_(far, corridor)]
    assert np.isfinite(zn).sum() > 50, 'ground band not populated'
    assert np.isfinite(zf).sum() > 50, 'plateau band not populated'

    measured = float(np.nanmedian(zf) - np.nanmedian(zn))
    assert measured == pytest.approx(PLATEAU_Z - GROUND_Z, abs=1e-3), (
        f'map reports a {measured:+.4f} m step for a true '
        f'{PLATEAU_Z - GROUND_Z:+.4f} m one')


def test_missing_pose_yields_no_points():
    """Without a pose the mapper must return nothing rather than dumping
    sensor-frame points at the origin, which would look like a working map
    full of rubbish."""
    node = build_node()
    node.pose = None
    assert node._to_world(np.zeros((3, 3)), 'livox_frame') is None
