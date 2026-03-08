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


class ROSBridge:
    """Thin wrapper around rclpy for odometry reading and thruster publishing.

    Falls back gracefully when rclpy is unavailable (offline testing).
    """

    def __init__(self, thrust_scale: float = THRUST_SCALE):
        self._thrust_scale = thrust_scale

        if not _HAS_ROS:
            print("[ROSBridge] rclpy not available — running in offline mode")
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
        """Convert Odometry message to 8D state vector."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular

        yaw = _quaternion_to_yaw(q)
        self._latest_state = np.array([
            p.x, p.y, p.z, yaw,
            v.x, v.y, v.z, w.z,
        ])

    # ── Public API ───────────────────────────────────────────────────────────

    def get_vehicle_state(self) -> np.ndarray | None:
        """Return 8D state [x,y,z,psi, dx,dy,dz,dpsi] or None."""
        return self._latest_state

    def send_thruster_command(self, surge: float, sway: float,
                              heave: float, yaw: float):
        """Apply mixing matrix, scale for Stonefish, and publish."""
        if self._node is None:
            return
        values = thruster_mixing(surge, sway, heave, yaw)
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
