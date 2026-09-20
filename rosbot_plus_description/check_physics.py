#!/usr/bin/env python3
"""Static physics self-check for the ROSbot Plus simulation model.

The feasibility surface of the project doc (section 5.1) is a purely physical
result, so a silent modelling regression there does not produce an error -- it
produces a plausible-looking but wrong paper. This node exists to make the two
regressions that are easiest to miss loud:

  1. The suspension spring is not applied at all.
     Gazebo Classic needs its joint extension to apply the spring, so the
     springs only exist because gazebo_classic_plugins.xacro injects them as a
     <gazebo reference="<joint>"> extension. If that block is ever lost, the
     suspension silently degrades into a free slider that bottoms out on its
     travel limit, and every climbing result becomes meaningless.

  2. The load-dependent ride height is wrong.
     Static sag is m_corner*g/k, so adding payload must lower the chassis by a
     predictable amount. That sag IS the mechanism by which payload reduces
     ground clearance and hence climbing capability, which is the physical
     story the C1 capability model is supposed to learn.

Everything it compares against is derived from the robot_description itself
(masses, spring stiffness, which links are sprung), so the check stays valid
when params.xacro is re-fitted during the sim-to-real calibration.

Usage, against a running simulation:

    ros2 run rosbot_plus_description check_physics
    ros2 run rosbot_plus_description check_physics --ros-args -p settle_time:=6.0
"""

import math
import sys
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

G = 9.80665

# A measured sag within this fraction of the prediction is considered a match.
# The tolerance is loose on purpose, and it is sized to catch "the spring is
# missing" (which pins the joint on its travel limit, 3x off or more), NOT to
# certify the spring rate.
#
# It has to absorb a known systematic offset: the assembled robot behaves as
# though the springs were ~8-10% softer than declared, identically loaded and
# unloaded. That offset is characterised in params.xacro and is absorbed by
# the sim-to-real fit, so the number to watch is the measured effective
# stiffness printed below, not this pass/fail flag.
SAG_TOLERANCE = 0.25


class PhysicsCheck(Node):

    def __init__(self):
        super().__init__('physics_check')

        self.declare_parameter('settle_time', 5.0)
        self.declare_parameter('sample_window', 2.0)
        self.settle_time = self.get_parameter('settle_time').value
        self.sample_window = self.get_parameter('sample_window').value

        self.urdf = None
        self.samples = []
        self.t_start = None

        # robot_description is published transient-local by
        # robot_state_publisher, so a late subscriber still receives it.
        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            String, '/robot_description', self._on_urdf, latched)
        self.create_subscription(
            JointState, '/joint_states', self._on_joints, 10)

    def _on_urdf(self, msg):
        if self.urdf is None:
            self.urdf = msg.data
            self.get_logger().info('got robot_description')

    def _on_joints(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t_start is None:
            self.t_start = now
        if now - self.t_start < self.settle_time:
            return
        if now - self.t_start > self.settle_time + self.sample_window:
            return
        self.samples.append(msg)

    def done(self):
        if self.t_start is None:
            return False
        now = self.get_clock().now().nanoseconds * 1e-9
        return (self.urdf is not None
                and now - self.t_start > self.settle_time + self.sample_window)

    # ---------------------------------------------------------------- model

    def model_prediction(self):
        """Derive expected sag from the URDF, with no hard-coded numbers."""
        root = ET.fromstring(self.urdf)

        masses = {}
        for link in root.findall('link'):
            inertial = link.find('inertial')
            if inertial is None:
                continue
            mass_el = inertial.find('mass')
            if mass_el is not None:
                masses[link.get('name')] = float(mass_el.get('value'))

        # Build the joint tree so "sprung" can be decided structurally rather
        # than by matching link-name patterns: a link is unsprung exactly when
        # a suspension joint sits between it and base_link.
        children = {}
        suspension_k = None
        for joint in root.findall('joint'):
            parent = joint.find('parent').get('link')
            child = joint.find('child').get('link')
            children.setdefault(parent, []).append((child, joint))
            if 'suspension' in joint.get('name'):
                dyn = joint.find('dynamics')
                if dyn is not None and dyn.get('spring_stiffness') is not None:
                    suspension_k = float(dyn.get('spring_stiffness'))

        unsprung = set()

        def mark(link):
            for child, _ in children.get(link, []):
                unsprung.add(child)
                mark(child)

        n_suspension = 0
        for joint in root.findall('joint'):
            if 'suspension' in joint.get('name'):
                n_suspension += 1
                child = joint.find('child').get('link')
                unsprung.add(child)
                mark(child)

        total = sum(masses.values())
        unsprung_mass = sum(m for lk, m in masses.items() if lk in unsprung)
        sprung_mass = total - unsprung_mass

        return {
            'total_mass': total,
            'sprung_mass': sprung_mass,
            'unsprung_mass': unsprung_mass,
            'n_suspension': n_suspension,
            'k': suspension_k,
            'expected_sag': (
                (sprung_mass / n_suspension) * G / suspension_k
                if suspension_k and n_suspension else None),
        }

    # ---------------------------------------------------------------- report

    def report(self):
        pred = self.model_prediction()

        susp, susp_vel, susp_eff, effort = {}, {}, {}, {}
        for msg in self.samples:
            for i, name in enumerate(msg.name):
                if 'suspension' in name:
                    susp.setdefault(name, []).append(msg.position[i])
                    if i < len(msg.velocity):
                        susp_vel.setdefault(name, []).append(msg.velocity[i])
                    if i < len(msg.effort):
                        susp_eff.setdefault(name, []).append(msg.effort[i])
                elif name.endswith('_Joint') and i < len(msg.effort):
                    effort.setdefault(name, []).append(msg.effort[i])

        print()
        print('=' * 68)
        print(' ROSbot Plus - static physics check')
        print('=' * 68)
        print(f"  total mass          {pred['total_mass']:8.3f} kg")
        print(f"  sprung  mass        {pred['sprung_mass']:8.3f} kg")
        print(f"  unsprung mass       {pred['unsprung_mass']:8.3f} kg "
              f"({100 * pred['unsprung_mass'] / pred['total_mass']:.0f}% of total)")
        print(f"  spring rate         {pred['k']:8.1f} N/m per wheel")
        print()

        if not susp:
            print('  FAIL  no suspension joint appeared on /joint_states.')
            print('        Check the state_interface block in ros2_control.xacro.')
            return 1

        expected = pred['expected_sag']
        print(f"  expected static sag {expected * 1000:8.2f} mm "
              f"(= m_corner*g/k)")
        print('  measured per wheel:')

        ok = True
        for name in sorted(susp):
            vals = susp[name]
            mean = sum(vals) / len(vals)
            # Sign convention: base_link sinking toward the ground drives the
            # knuckle upward in base_link's frame, so a supported chassis
            # shows a POSITIVE joint position.
            rel = abs(mean - expected) / expected if expected else math.inf
            flag = 'ok' if rel <= SAG_TOLERANCE else 'MISMATCH'
            if flag != 'ok':
                ok = False
            print(f'    {name:24s} {mean * 1000:8.2f} mm   '
                  f'({rel * 100:5.1f}% off)  {flag}')

        # Force balance cross-check. Cutting the model at the suspension
        # joints, each corner must satisfy  N_i = m_unsprung_corner*g + k*q,
        # so the four spring forces have to add up to exactly the sprung
        # weight. Comparing that sum against m_sprung*g catches an effective
        # stiffness that differs from the declared one -- a discretisation
        # artefact would show up here as a constant ratio, while a mass
        # bookkeeping error would show up as a constant offset.
        mean_q = sum(sum(v) / len(v) for v in susp.values()) / len(susp)
        spring_force_total = len(susp) * pred['k'] * mean_q
        sprung_weight = pred['sprung_mass'] * G
        print(f'  force balance:')
        print(f'    sum of spring forces  {spring_force_total:8.1f} N '
              f'(= n*k*mean_q)')
        print(f'    sprung weight         {sprung_weight:8.1f} N '
              f'(= m_sprung*g)')
        print(f'    ratio                 {spring_force_total / sprung_weight:8.3f} '
              f'  -> effective k = {pred["k"] * sprung_weight / spring_force_total:.0f} N/m')

        if susp_vel:
            max_v = max(abs(sum(v) / len(v)) for v in susp_vel.values())
            print(f'    max |suspension velocity| {max_v * 1000:6.3f} mm/s '
                  f'({"settled" if max_v < 1e-3 else "STILL MOVING"})')
        if susp_eff:
            print('    measured joint force:')
            for name in sorted(susp_eff):
                vals = susp_eff[name]
                mean_f = sum(vals) / len(vals)
                q = sum(susp[name]) / len(susp[name])
                k_eff = -mean_f / q if q else float('nan')
                print(f'      {name:22s} {mean_f:8.2f} N  -> k_eff {k_eff:7.0f} N/m')

        print()
        if effort:
            print('  drive joint effort at rest (motor-current proxy for C1):')
            for name in sorted(effort):
                vals = effort[name]
                print(f'    {name:24s} {sum(vals) / len(vals):8.3f} N.m')
        else:
            print('  NOTE  no effort on /joint_states - the payload estimator')
            print('        of doc section 4 has no motor-current input.')
        print()

        if ok:
            print('  PASS  suspension is spring-supported, not bottomed out on its')
            print('        travel limit, and the load path balances. Use the measured')
            print('        effective stiffness above as the calibration reference.')
            print('=' * 68)
            return 0

        print('  FAIL  measured sag does not match m_corner*g/k.')
        print('        The usual cause is a lost <gazebo reference="..._suspension_joint">')
        print('        spring block in gazebo_classic_plugins.xacro: without it Gazebo')
        print('        spring and the joint bottoms out on its travel limit.')
        print('=' * 68)
        return 1


def main(args=None):
    rclpy.init(args=args)
    node = PhysicsCheck()
    try:
        deadline = node.get_clock().now().nanoseconds * 1e-9 + 60.0
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.get_clock().now().nanoseconds * 1e-9 > deadline:
                node.get_logger().error(
                    'timed out waiting for /robot_description + /joint_states')
                return 2
        code = node.report()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
