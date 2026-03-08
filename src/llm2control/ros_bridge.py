"""ROS2 bridge: subscribe to odometry, publish thruster commands."""

import math
import numpy as np

from llm2control.config import THRUSTER_TOPIC, ODOMETRY_TOPIC, THRUST_SCALE
from llm2control.dynamics import thruster_mixing

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray
    from nav_msgs.msg import Odometry
    _HAS_ROS = True
except ImportError:
    _HAS_ROS = False


def _quaternion_to_yaw(q) -> float:
    """Extract yaw from quaternion (x, y, z, w)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quaternion_to_roll(q) -> float:
    """Extract roll from quaternion (x, y, z, w)."""
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    return math.atan2(sinr_cosp, cosr_cosp)


class ROSBridge:
    """Thin wrapper around rclpy for odometry reading and thruster publishing.

    Falls back gracefully when rclpy is unavailable (offline testing).
    """

    # EMA smoothing factor for velocity components (0 = full filter, 1 = no filter)
    VELOCITY_ALPHA = 0.3

    def __init__(self, thrust_scale: float = THRUST_SCALE):
        self._thrust_scale = thrust_scale

        if not _HAS_ROS:
            self._node = None
            self._latest_state = None
            return

        if not rclpy.ok():
            rclpy.init()

        self._node = Node("llm2control_bridge")
        self._latest_state: np.ndarray | None = None

        # Subscriber
        self._node.create_subscription(
            Odometry, ODOMETRY_TOPIC, self._odom_cb, 10
        )

        # Publisher
        self._thruster_pub = self._node.create_publisher(
            Float64MultiArray, THRUSTER_TOPIC, 10
        )

        self._node.get_logger().info(
            f"ROSBridge ready (thrust_scale={self._thrust_scale})")

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: "Odometry"):
        """Convert Odometry message to 10D state vector (world frame).

        Velocity components are EMA-filtered to reduce noise-induced jitter.
        """
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear   # body-frame twist
        w = msg.twist.twist.angular

        roll = _quaternion_to_roll(q)
        yaw = _quaternion_to_yaw(q)

        # Convert body-frame velocity to world-frame
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        vx_world = v.x * cos_yaw - v.y * sin_yaw
        vy_world = v.x * sin_yaw + v.y * cos_yaw

        raw_vel = np.array([vx_world, vy_world, v.z, w.x, w.z])

        # EMA filter on velocities
        if self._latest_state is not None:
            prev_vel = self._latest_state[5:10]
            a = self.VELOCITY_ALPHA
            filtered_vel = a * raw_vel + (1.0 - a) * prev_vel
        else:
            filtered_vel = raw_vel

        self._latest_state = np.array([
            p.x, p.y, p.z, roll, yaw,
            filtered_vel[0], filtered_vel[1], filtered_vel[2],
            filtered_vel[3], filtered_vel[4],
        ])

    # ── Public API ───────────────────────────────────────────────────────────

    @property
    def is_offline(self) -> bool:
        """True if rclpy was not available (no ROS2 connection)."""
        return self._node is None

    def get_vehicle_state(self) -> np.ndarray | None:
        """Return 10D state [x,y,z,phi,psi, dx,dy,dz,dphi,dpsi] or None."""
        return self._latest_state

    def send_thruster_command(self, surge: float, sway: float,
                              heave: float, roll: float, yaw: float):
        """Apply mixing matrix, scale for Stonefish, and publish."""
        if self._node is None:
            return
        values = thruster_mixing(surge, sway, heave, roll, yaw)
        scaled = [v * self._thrust_scale for v in values]
        msg = Float64MultiArray()
        msg.data = scaled
        self._thruster_pub.publish(msg)

    def spin_once(self, timeout_sec: float = 0.01):
        if self._node is not None:
            rclpy.spin_once(self._node, timeout_sec=timeout_sec)

    def zero_thrust(self):
        """Publish zero thrust to all thrusters (safety stop)."""
        if self._node is None:
            return
        msg = Float64MultiArray()
        msg.data = [0.0] * 8
        self._thruster_pub.publish(msg)

    def shutdown(self):
        if self._node is not None:
            self.zero_thrust()
            self._node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
