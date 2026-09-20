#!/usr/bin/env python3
"""Drop non-finite points from the simulated lidar cloud.

Gazebo Classic's gpu_ray sensor can report a ray that hits nothing as an
infinite range, which becomes a point with inf coordinates. On
the ROSbot Plus this is not a rare edge case: the Mid-360's vertical FOV runs
from -7 deg to +52 deg, so most of the beam points upward, and in an outdoor
world with nothing overhead the great majority of rays return nothing at all.
Measured on the flat world: 22800 of 25600 points, 89%, non-finite.

That matters because FAST-LIO's default_handler admits a point when

    x*x + y*y + z*z > blind*blind

which is TRUE for infinity. So every sky ray is accepted as a valid
measurement, the ikd-Tree fills with infinities, and the filter never
converges: /Laser_map keeps ticking on its timer while /Odometry, /path and
/cloud_registered stay silent forever, with no error message anywhere.

The real Livox driver does not emit these points, so this node is a
simulation-only shim. It is deliberately NOT a patch to FAST_LIO, which is kept
as a clean upstream checkout.

    ros2 run rosbot_plus_description cloud_sanitize --ros-args \
        -r ~/input:=/mid360/points -r ~/output:=/mid360/points_clean
"""

import sys

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def _quat_to_matrix(q):
    """Rotation matrix from a geometry_msgs Quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class CloudSanitize(Node):

    def __init__(self):
        super().__init__('cloud_sanitize')

        self.declare_parameter('input_topic', '/mid360/points')
        self.declare_parameter('output_topic', '/mid360/points_clean')
        # Points closer than this are the robot seeing itself. FAST-LIO applies
        # its own `blind` too, but dropping them here keeps the republished
        # cloud honest for anything else that consumes it (grid_map, RViz).
        self.declare_parameter('min_range', 0.3)
        self.declare_parameter('max_range', 40.0)

        # Self-observation filter, in base_link coordinates.
        #
        # A RANGE threshold cannot do this job once the lidar is tilted. At
        # 25 deg down the beam grazes the robot's own chassis roof: the body
        # reaches x = 0.648 in base_link while the lidar sits at x = 0.265, so
        # the front of the robot is 0.37 m away - already past a 0.3 m minimum
        # range. Raising min_range until the chassis is excluded would also
        # discard everything inside 0.65 m, which is precisely the near-ground
        # coverage the tilt was adopted to gain. Measured consequence of NOT
        # filtering: the height difference across a 0.11 m step read +0.56 m.
        #
        # The box below covers chassis, wheels and a payload cube, plus a
        # margin. Derived from rosbot_plus.xacro: chassis x -0.118..0.648,
        # |y| <= 0.175, z 0.120..0.319; wheels out to |y| = 0.335 and down to
        # z = 0; payload cube up to z = 0.619.
        self.declare_parameter('self_filter', True)
        self.declare_parameter('self_box_min', [-0.25, -0.40, -0.05])
        self.declare_parameter('self_box_max', [0.78, 0.40, 0.66])
        self.declare_parameter('base_frame', 'base_link')

        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.self_filter = bool(self.get_parameter('self_filter').value)
        self.box_min = np.array(self.get_parameter('self_box_min').value)
        self.box_max = np.array(self.get_parameter('self_box_max').value)
        self.base_frame = self.get_parameter('base_frame').value
        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        # The sensor-to-base transform is static, so it is looked up once and
        # cached. Reading it from TF rather than restating it keeps the filter
        # correct at any lidar_tilt_deg without a second place to update.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._R = None
        self._t = None
        self.n_self = 0

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(PointCloud2, out_topic, qos)
        self.create_subscription(PointCloud2, in_topic, self._on_cloud, qos)

        self.reported = False
        # Received and published are counted separately on purpose. The first
        # version of this node logged only on receipt, which proved the
        # subscription worked and said nothing about the publish - and when the
        # downstream stack went silent there was no way to tell which half had
        # failed.
        self.n_in = 0
        self.n_out = 0
        self.get_logger().info(f'sanitising {in_topic} -> {out_topic}')

    # Counted from inside the callback rather than on a timer: a timer created
    # under use_sim_time never fires until /clock arrives, so a heartbeat timer
    # is silent in exactly the situation where the heartbeat is wanted.

    def _on_cloud(self, msg):
        self.n_in += 1
        # Read the raw buffer as a structured array rather than going through
        # point_cloud2.read_points: at 25600 points and 10 Hz the per-point
        # Python loop costs more than the whole rest of the pipeline.
        dtype, names = self._dtype_from_fields(msg)
        if dtype is None:
            return
        arr = np.frombuffer(msg.data, dtype=dtype)

        xyz = np.stack([arr['x'], arr['y'], arr['z']], axis=-1)
        finite = np.isfinite(xyz).all(axis=1)
        r = np.linalg.norm(np.where(finite[:, None], xyz, 0.0), axis=1)
        keep = finite & (r >= self.min_range) & (r <= self.max_range)

        if self.self_filter:
            inside = self._inside_robot(xyz, msg.header.frame_id)
            if inside is not None:
                self.n_self += int((keep & inside).sum())
                keep = keep & ~inside

        out = arr[keep]
        if not self.reported:
            self.get_logger().info(
                f'first cloud: {len(arr)} points in, {len(out)} kept '
                f'({100.0 * (len(arr) - len(out)) / max(len(arr), 1):.0f}% dropped '
                f'as non-finite or out of range)')
            self.reported = True

        cleaned = PointCloud2()
        cleaned.header = msg.header
        cleaned.height = 1
        cleaned.width = len(out)
        cleaned.fields = msg.fields
        cleaned.is_bigendian = msg.is_bigendian
        cleaned.point_step = msg.point_step
        cleaned.row_step = msg.point_step * len(out)
        cleaned.data = out.tobytes()
        # Every remaining point is finite by construction, which lets
        # downstream consumers skip their own NaN pass.
        cleaned.is_dense = True
        self.pub.publish(cleaned)
        self.n_out += 1
        if self.n_out % 50 == 0:
            self.get_logger().info(
                f'received {self.n_in}, published {self.n_out}, '
                f'{self.n_self} points dropped as self-observation')

    def _inside_robot(self, xyz, sensor_frame):
        """Boolean mask of points falling inside the robot's own body.

        Returns None until the static transform is available, so the node
        degrades to no self-filtering rather than to dropping everything - a
        filter that silently deleted the whole cloud because TF was late would
        look exactly like a dead sensor.
        """
        if self._R is None:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame, sensor_frame, rclpy.time.Time())
            except Exception:
                return None
            self._R = _quat_to_matrix(tf.transform.rotation)
            self._t = np.array([tf.transform.translation.x,
                                tf.transform.translation.y,
                                tf.transform.translation.z])
            self.get_logger().info(
                f'self-filter active: {sensor_frame} -> {self.base_frame}, '
                f'box {self.box_min.tolist()} .. {self.box_max.tolist()}')

        p = xyz @ self._R.T + self._t
        with np.errstate(invalid='ignore'):
            return np.all((p >= self.box_min) & (p <= self.box_max), axis=1)

    @staticmethod
    def _dtype_from_fields(msg):
        """Build a numpy dtype matching the message layout, padding included."""
        type_map = {
            PointField.INT8: 'i1', PointField.UINT8: 'u1',
            PointField.INT16: 'i2', PointField.UINT16: 'u2',
            PointField.INT32: 'i4', PointField.UINT32: 'u4',
            PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8',
        }
        names, formats, offsets = [], [], []
        for f in msg.fields:
            if f.datatype not in type_map:
                return None, None
            names.append(f.name)
            formats.append(type_map[f.datatype])
            offsets.append(f.offset)
        if not {'x', 'y', 'z'} <= set(names):
            return None, None
        # itemsize must be the full point_step so the gaps the bridge leaves
        # between fields are preserved when the array is written back out.
        return np.dtype({'names': names, 'formats': formats,
                         'offsets': offsets, 'itemsize': msg.point_step}), names


def main(args=None):
    rclpy.init(args=args)
    node = CloudSanitize()
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
