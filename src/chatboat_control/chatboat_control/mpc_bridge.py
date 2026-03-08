#!/usr/bin/env python3
"""MPC Bridge: PD controller that accepts goal poses and drives thrusters.

Subscribes to odometry and goal poses, computes world-frame PD control,
converts to body-frame thrust commands, and publishes scaled setpoints
for the Stonefish simulator.

No external dependencies beyond numpy + standard ROS2.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64MultiArray


# ── Inline helpers (avoid cross-package dependency on llm2control) ──────────


def _quaternion_to_yaw(q) -> float:
    """Extract yaw from quaternion (x, y, z, w)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _world_to_body(u_x: float, u_y: float, psi: float):
    """Convert world-frame XY command to body-frame surge/sway."""
    cos_psi = math.cos(psi)
    sin_psi = math.sin(psi)
    surge = u_x * cos_psi + u_y * sin_psi
    sway = -u_x * sin_psi + u_y * cos_psi
    return surge, sway


def _thruster_mixing(surge: float, sway: float, heave: float,
                     yaw: float) -> list[float]:
    """BlueROV2 Heavy 8-thruster mixing. Returns values in [-1, 1]."""
    t1 = surge + sway + yaw   # FrontRight
    t2 = surge - sway - yaw   # FrontLeft
    t3 = -surge + sway - yaw  # BackRight
    t4 = -surge - sway + yaw  # BackLeft
    t5 =   heave              # DFR starboard-front (inverted=false)
    t6 = -(heave)             # DFL port-front      (inverted=true, pre-negate)
    t7 = -(heave)             # DBR starboard-back  (inverted=true, pre-negate)
    t8 =   heave              # DBL port-back       (inverted=false)
    return [max(-1.0, min(1.0, t)) for t in [t1, t2, t3, t4, t5, t6, t7, t8]]


# ── Node ────────────────────────────────────────────────────────────────────


class MPCBridge(Node):
    def __init__(self):
        super().__init__('mpc_bridge')

        # Parameters
        self.declare_parameter('thrust_scale', 50.0)
        self.declare_parameter('kp_pos', 0.5)
        self.declare_parameter('kd_pos', 0.8)
        self.declare_parameter('kp_yaw', 0.3)
        self.declare_parameter('kd_yaw', 0.5)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('yaw_tolerance', 0.15)
        self.declare_parameter('max_accel', 0.3)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('odom_timeout', 1.0)

        self._thrust_scale = self.get_parameter('thrust_scale').value
        self._kp_pos = self.get_parameter('kp_pos').value
        self._kd_pos = self.get_parameter('kd_pos').value
        self._kp_yaw = self.get_parameter('kp_yaw').value
        self._kd_yaw = self.get_parameter('kd_yaw').value
        self._goal_tol = self.get_parameter('goal_tolerance').value
        self._yaw_tol = self.get_parameter('yaw_tolerance').value
        self._max_accel = self.get_parameter('max_accel').value
        control_rate = self.get_parameter('control_rate').value
        self._odom_timeout = self.get_parameter('odom_timeout').value

        # State
        self._state = None  # [x, y, z, yaw, dx, dy, dz, dyaw]
        self._odom_stamp = None
        self._goal = None   # [x, y, z, yaw]

        # Subscribers
        self.create_subscription(
            Odometry, '/chatboat/odometry', self._odom_cb, 10)
        self.create_subscription(
            PoseStamped, '/chatboat/mpc_bridge/goal', self._goal_cb, 10)

        # Publishers
        self._thrust_pub = self.create_publisher(
            Float64MultiArray, '/chatboat/thruster_commands', 10)
        self._reached_pub = self.create_publisher(
            Bool, '/chatboat/mpc_bridge/goal_reached', 10)

        # Control timer
        dt = 1.0 / control_rate
        self.create_timer(dt, self._control_loop)

        # Zero thrust on shutdown
        self.context.on_shutdown(self._zero_thrust)

        self.get_logger().info(
            f'MPC Bridge ready (thrust_scale={self._thrust_scale}, '
            f'kp_pos={self._kp_pos}, kd_pos={self._kd_pos})')

    # ── Callbacks ───────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v_body = msg.twist.twist.linear
        w = msg.twist.twist.angular

        yaw = _quaternion_to_yaw(q)

        # Convert body-frame velocity to world-frame
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        vx_world = v_body.x * cos_yaw - v_body.y * sin_yaw
        vy_world = v_body.x * sin_yaw + v_body.y * cos_yaw

        self._state = np.array([
            p.x, p.y, p.z, yaw,
            vx_world, vy_world, v_body.z, w.z,
        ])
        self._odom_stamp = self.get_clock().now()

    def _goal_cb(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation
        yaw = _quaternion_to_yaw(q)
        self._goal = np.array([p.x, p.y, p.z, yaw])
        self.get_logger().info(
            f'New goal: [{p.x:.2f}, {p.y:.2f}, {p.z:.2f}], yaw={yaw:.2f}')

    # ── Control loop ────────────────────────────────────────────────────────

    def _control_loop(self):
        # Safety: no odom yet
        if self._state is None or self._odom_stamp is None:
            self._zero_thrust()
            return

        # Safety: stale odom
        age = (self.get_clock().now() - self._odom_stamp).nanoseconds * 1e-9
        if age > self._odom_timeout:
            self.get_logger().warn(f'Stale odom ({age:.1f}s), zeroing thrust')
            self._zero_thrust()
            return

        # No goal: zero thrust (station-keeping could go here)
        if self._goal is None:
            self._zero_thrust()
            return

        pos = self._state[:3]       # x, y, z
        yaw = self._state[3]
        vel = self._state[4:7]      # vx, vy, vz (world frame)
        yaw_rate = self._state[7]

        goal_pos = self._goal[:3]
        goal_yaw = self._goal[3]

        # Position PD (world frame)
        pos_error = goal_pos - pos
        u_world = self._kp_pos * pos_error - self._kd_pos * vel
        # Clamp each axis
        u_world = np.clip(u_world, -self._max_accel, self._max_accel)

        # Yaw PD
        yaw_error = _wrap_angle(goal_yaw - yaw)
        u_yaw = self._kp_yaw * yaw_error - self._kd_yaw * yaw_rate
        u_yaw = max(-self._max_accel, min(self._max_accel, u_yaw))

        # World → body for horizontal
        surge, sway = _world_to_body(u_world[0], u_world[1], yaw)
        heave = u_world[2]

        # Mixing → [-1, 1]
        thrusts = _thruster_mixing(surge, sway, heave, u_yaw)

        # Scale for Stonefish
        max_setpoint = 1000.0
        scaled = [max(-max_setpoint, min(max_setpoint, t * self._thrust_scale))
                  for t in thrusts]

        # Publish thrust
        msg = Float64MultiArray()
        msg.data = scaled
        self._thrust_pub.publish(msg)

        # Check goal reached
        pos_dist = np.linalg.norm(pos_error)
        yaw_dist = abs(yaw_error)
        reached = pos_dist < self._goal_tol and yaw_dist < self._yaw_tol

        reached_msg = Bool()
        reached_msg.data = reached
        self._reached_pub.publish(reached_msg)

        if reached:
            self.get_logger().info(
                f'Goal reached (dist={pos_dist:.3f}m, yaw_err={yaw_dist:.3f}rad)',
                throttle_duration_sec=2.0)

    # ── Helpers ─────────────────────────────────────────────────────────────

    def _zero_thrust(self):
        msg = Float64MultiArray()
        msg.data = [0.0] * 8
        self._thrust_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MPCBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
