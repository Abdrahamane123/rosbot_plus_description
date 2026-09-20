#!/usr/bin/env python3
"""Rolling elevation map built from FAST-LIO's registered point cloud.

This is the layer the project doc's architecture (section 4) puts between the
LIO and the C4 unified step/ramp/groove detector. It is a CPU implementation:
the doc names elevation_mapping_cupy, which needs CUDA, and this machine has an
Intel iGPU.

Why a MAP and not just the current scan - the sensor cannot see the ground in
front of the robot at all. The Mid-360's vertical FOV starts at -7 deg, so with
the lidar at 0.349 m the nearest possible ground return is

    0.349 / tan(7 deg) = 2.84 m

Everything closer is blind. A step is therefore observed from ~3 m away and
then DISAPPEARS from view during the final approach - exactly when the
capability decision has to be made. So the height has to be remembered, which
is what this node is for.

That blind radius scales directly with mount height (8.14 x it), so lowering
the lidar to 0.20 m would shrink it to 1.63 m. Worth weighing against occlusion
by the robot's own body when the real mount is designed.

Layers published:
    elevation      highest point in the cell. Conservative for obstacles: a
                   step must not be averaged away by ground points sharing the
                   cell.
    elevation_min  lowest point. Together with `elevation` this is what makes a
                   groove visible - a cell straddling a groove edge has a large
                   spread, while flat ground has almost none.
    spread         elevation - elevation_min, i.e. within-cell height range.
                   The cheapest discontinuity cue there is, and the one C4 can
                   threshold before doing anything more expensive.
    n_points       observation count, so a cell seen once is distinguishable
                   from a cell seen a hundred times. Uncertainty in the doc's
                   P(success | ...) has to come from somewhere.

    ros2 run rosbot_plus_description elevation_map --ros-args -p use_sim_time:=true
"""

import sys

import numpy as np
import rclpy
import tf2_ros
from grid_map_msgs.msg import GridMap, GridMapInfo
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


class ElevationMap(Node):

    def __init__(self):
        super().__init__('elevation_map')

        self.declare_parameter('cloud_topic', '/cloud_registered')
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('map_frame', 'camera_init')

        # Register the cloud against the supplied pose instead of trusting it
        # to arrive pre-registered.
        #
        # /cloud_registered is already in FAST-LIO's frame, drift included, so
        # pointing `odom_topic` at ground truth would re-centre the map while
        # leaving every point where the drift put it - a half-measure that
        # looks like a fix. For a SENSOR-GEOMETRY experiment the drift has to
        # go entirely: set this true and feed the sensor-frame cloud
        # (/mid360/points_clean) with odom_topic:=/ground_truth/odom, and the
        # only thing varying between runs is the sensor.
        #
        # Leave it false for anything meant to represent the real robot, which
        # has no ground truth and must live with its own drift.
        self.declare_parameter('register_from_pose', False)
        self.declare_parameter('sensor_frame', 'livox_frame')
        self.declare_parameter('base_frame', 'base_footprint')
        # 0.05 m puts two cells across the narrowest groove that matters
        # (0.12 m) and about two across the 0.11 m step boundary. Finer costs
        # memory quadratically and gains nothing while the cloud is this sparse.
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('length', 12.0)
        self.declare_parameter('publish_rate', 2.0)

        self.register = bool(self.get_parameter('register_from_pose').value)
        self.sensor_frame = self.get_parameter('sensor_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.pose = None                 # latest odom pose, when registering
        self._R_sb = None                # sensor -> base, static
        self._t_sb = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.res = self.get_parameter('resolution').value
        self.length = self.get_parameter('length').value
        self.map_frame = self.get_parameter('map_frame').value

        self.n = int(round(self.length / self.res))
        # Robot-centric: the map recentres as the robot moves, so memory stays
        # bounded on a long run. Centre starts unset and follows odometry.
        self.center = np.zeros(2)
        self.have_center = False

        self.max_z = np.full((self.n, self.n), np.nan, dtype=np.float32)
        self.min_z = np.full((self.n, self.n), np.nan, dtype=np.float32)
        self.count = np.zeros((self.n, self.n), dtype=np.float32)

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            PointCloud2, self.get_parameter('cloud_topic').value,
            self._on_cloud, qos)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self._on_odom, 10)
        self.pub = self.create_publisher(GridMap, '/elevation_map', 1)
        self.create_timer(1.0 / self.get_parameter('publish_rate').value,
                          self._publish)
        self.reported = False

    # ------------------------------------------------------------------ input

    @staticmethod
    def _quat_matrix(q):
        x, y, z, w = q.x, q.y, q.z, q.w
        return np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
            [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])

    def _on_odom(self, msg):
        if self.register:
            # When registering from a pose, the map lives in THAT pose's frame,
            # not in the LIO frame. Publishing it as camera_init anyway would
            # invite consumers to correct for a LIO offset that is no longer
            # there - which is exactly the error the tilt probe made.
            if msg.header.frame_id:
                self.map_frame = msg.header.frame_id
            self.pose = (self._quat_matrix(msg.pose.pose.orientation),
                         np.array([msg.pose.pose.position.x,
                                   msg.pose.pose.position.y,
                                   msg.pose.pose.position.z]))
        p = msg.pose.pose.position
        new_center = np.array([p.x, p.y])
        if not self.have_center:
            self.center = new_center
            self.have_center = True
            return
        # Recentre only in whole cells, so the grid never resamples itself:
        # shifting by a fraction of a cell would require interpolation and
        # would slowly smear real height edges into ramps, which is precisely
        # the distinction C4 has to make.
        shift = np.round((new_center - self.center) / self.res).astype(int)
        if np.any(shift != 0):
            self._roll(shift)
            self.center = self.center + shift * self.res

    def _roll(self, shift):
        """Shift the grid by whole cells, clearing what scrolls in."""
        sx, sy = int(shift[0]), int(shift[1])
        for arr, fill in ((self.max_z, np.nan), (self.min_z, np.nan),
                          (self.count, 0.0)):
            arr[:] = np.roll(arr, (-sx, -sy), axis=(0, 1))
            if sx > 0:
                arr[-sx:, :] = fill
            elif sx < 0:
                arr[:-sx, :] = fill
            if sy > 0:
                arr[:, -sy:] = fill
            elif sy < 0:
                arr[:, :-sy] = fill

    def _on_cloud(self, msg):
        if not self.have_center:
            return
        pts = self._read_xyz(msg)
        if pts is None or len(pts) == 0:
            return

        if self.register:
            pts = self._to_world(pts, msg.header.frame_id)
            if pts is None:
                return

        # Cell indices, robot-centric.
        rel = pts[:, :2] - self.center
        idx = np.floor(rel / self.res).astype(int) + self.n // 2
        inside = ((idx[:, 0] >= 0) & (idx[:, 0] < self.n)
                  & (idx[:, 1] >= 0) & (idx[:, 1] < self.n))
        idx, z = idx[inside], pts[inside, 2]
        if len(z) == 0:
            return

        flat = idx[:, 0] * self.n + idx[:, 1]
        # np.maximum.at is the only correct way to fold duplicates: several
        # thousand points land in the same cell each scan, and plain fancy
        # indexing would keep whichever happened to be written last rather
        # than the highest.
        cur_max = self.max_z.reshape(-1)
        cur_min = self.min_z.reshape(-1)
        cnt = self.count.reshape(-1)

        fresh = np.isnan(cur_max[flat])
        if np.any(fresh):
            first = flat[fresh]
            cur_max[first] = -np.inf
            cur_min[first] = np.inf

        np.maximum.at(cur_max, flat, z)
        np.minimum.at(cur_min, flat, z)
        np.add.at(cnt, flat, 1.0)

        if not self.reported:
            self.get_logger().info(
                f'first cloud mapped: {len(z)} points into '
                f'{len(np.unique(flat))} cells of {self.res} m')
            self.reported = True

    def _to_world(self, pts, frame):
        """Sensor-frame points -> world, via the supplied pose.

        Returns None until both the static sensor->base transform and a pose
        are available, so the map stays empty rather than accumulating points
        at the origin - which would look like a working map full of rubbish.
        """
        if self.pose is None:
            return None
        if self._R_sb is None:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame, frame, rclpy.time.Time())
            except Exception:
                return None
            self._R_sb = self._quat_matrix(tf.transform.rotation)
            self._t_sb = np.array([tf.transform.translation.x,
                                   tf.transform.translation.y,
                                   tf.transform.translation.z])
            self.get_logger().info(
                f'registering from pose: {frame} -> {self.base_frame} -> world')
        R_wb, t_wb = self.pose
        return (pts @ self._R_sb.T + self._t_sb) @ R_wb.T + t_wb

    @staticmethod
    def _read_xyz(msg):
        type_map = {PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8',
                    PointField.INT32: 'i4', PointField.UINT32: 'u4',
                    PointField.INT16: 'i2', PointField.UINT16: 'u2',
                    PointField.INT8: 'i1', PointField.UINT8: 'u1'}
        names, formats, offsets = [], [], []
        for f in msg.fields:
            if f.datatype not in type_map:
                return None
            names.append(f.name)
            formats.append(type_map[f.datatype])
            offsets.append(f.offset)
        if not {'x', 'y', 'z'} <= set(names):
            return None
        dt = np.dtype({'names': names, 'formats': formats,
                       'offsets': offsets, 'itemsize': msg.point_step})
        a = np.frombuffer(msg.data, dtype=dt)
        pts = np.stack([a['x'], a['y'], a['z']], axis=-1).astype(np.float64)
        return pts[np.isfinite(pts).all(axis=1)]

    # ----------------------------------------------------------------- output

    def _publish(self):
        if not self.have_center:
            return
        msg = GridMap()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        info = GridMapInfo()
        info.resolution = float(self.res)
        info.length_x = float(self.length)
        info.length_y = float(self.length)
        info.pose.position.x = float(self.center[0])
        info.pose.position.y = float(self.center[1])
        info.pose.orientation.w = 1.0
        msg.info = info

        spread = self.max_z - self.min_z
        layers = {
            'elevation': self.max_z,
            'elevation_min': self.min_z,
            'spread': spread,
            'n_points': np.where(self.count > 0, self.count, np.nan),
        }
        msg.layers = list(layers)
        msg.basic_layers = ['elevation']
        for name in msg.layers:
            msg.data.append(self._to_multiarray(layers[name]))
        msg.outer_start_index = 0
        msg.inner_start_index = 0
        self.pub.publish(msg)

    @staticmethod
    def _to_multiarray(grid):
        """grid_map stores column-major with rows/columns reversed vs numpy."""
        m = Float32MultiArray()
        d0 = MultiArrayDimension()
        d0.label, d0.size = 'column_index', grid.shape[1]
        d0.stride = grid.shape[0] * grid.shape[1]
        d1 = MultiArrayDimension()
        d1.label, d1.size = 'row_index', grid.shape[0]
        d1.stride = grid.shape[0]
        m.layout.dim = [d0, d1]
        m.data = grid[::-1, ::-1].T.reshape(-1).astype(np.float32).tolist()
        return m


def main(args=None):
    rclpy.init(args=args)
    node = ElevationMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
