import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool


class State:
    INIT = 'INIT'
    APPROACH_CUBE_A = 'APPROACH_CUBE_A'
    DESCEND_TO_CUBE_A = 'DESCEND_TO_CUBE_A'
    GRIP_CUBE_A = 'GRIP_CUBE_A'
    LIFT_CUBE_A = 'LIFT_CUBE_A'
    TRANSIT_TO_CUBE_B = 'TRANSIT_TO_CUBE_B'
    DESCEND_TO_CUBE_B = 'DESCEND_TO_CUBE_B'
    RELEASE_CUBE_A = 'RELEASE_CUBE_A'
    ASCEND = 'ASCEND'
    DONE = 'DONE'


class TestCommander(Node):
    """State-machine demo: pick CubeA, stack on CubeB.

    Uses simple waypoint-based open-loop commands to demonstrate
    the ROS2 interface works. Phase 2 will replace this with
    MPC-based task-priority control.

    Robot: BlueROV2 Heavy (8 thrusters) + Reach Alpha 5 (4-DOF arm)
    """

    # Target positions (NED frame)
    CUBE_A_POS = (1.0, 0.0, 4.95)
    CUBE_B_POS = (2.0, 0.0, 4.95)
    CRUISE_DEPTH = 3.5
    APPROACH_DEPTH = 4.5

    # Arm configurations (axis_e, axis_d, axis_c, axis_b)
    ARM_STOWED = [0.0, 0.0, 0.0, 0.0]
    ARM_REACH_DOWN = [0.0, 1.2, 1.0, 0.5]

    def __init__(self):
        super().__init__('test_commander')

        # Publishers
        self._thruster_pub = self.create_publisher(
            Float64MultiArray, '/chatboat/thruster_commands', 10)
        self._joint_pub = self.create_publisher(
            JointState, '/chatboat/joint_commands', 10)

        # Subscribers
        self._odom_sub = self.create_subscription(
            Odometry, '/chatboat/odometry', self._odom_cb, 10)
        self._joint_sub = self.create_subscription(
            JointState, '/chatboat/joint_states', self._joint_state_cb, 10)

        # Gripper service client
        self._gripper_client = self.create_client(
            SetBool, '/chatboat/gripper/activate')

        # State
        self._state = State.INIT
        self._odom = None
        self._joint_positions = [0.0] * 4
        self._state_entry_time = self.get_clock().now()
        self._state_duration = 5.0  # seconds per state (open-loop timing)

        # Main timer
        self._timer = self.create_timer(0.1, self._tick)

        self.get_logger().info('Test commander started - cube stacking demo')

    def _odom_cb(self, msg):
        self._odom = msg

    def _joint_state_cb(self, msg):
        if len(msg.position) >= 4:
            self._joint_positions = list(msg.position[:4])

    def _send_thruster(self, surge=0.0, sway=0.0, heave=0.0, yaw=0.0):
        """BlueROV2 Heavy vectored 8-thruster mixing.

        Horizontal thrusters at +/-45 deg angles:
          T1 FrontRight  (inverted)
          T2 FrontLeft   (inverted)
          T3 BackRight
          T4 BackLeft
        Vertical thrusters:
          T5 DiveFrontRight
          T6 DiveFrontLeft  (inverted)
          T7 DiveBackRight  (inverted)
          T8 DiveBackLeft
        """
        msg = Float64MultiArray()
        # Horizontal thrusters at +-45 deg angles
        t1 = surge + sway + yaw    # FrontRight
        t2 = surge - sway - yaw    # FrontLeft
        t3 = -surge + sway - yaw   # BackRight
        t4 = -surge - sway + yaw   # BackLeft
        # Vertical thrusters (equal for pure heave)
        t5 = heave                 # DiveFrontRight
        t6 = heave                 # DiveFrontLeft
        t7 = heave                 # DiveBackRight
        t8 = heave                 # DiveBackLeft
        # Clamp to [-1, 1]
        msg.data = [max(-1.0, min(1.0, t)) for t in [t1, t2, t3, t4, t5, t6, t7, t8]]
        self._thruster_pub.publish(msg)

    def _send_arm(self, positions):
        msg = JointState()
        msg.name = ['axis_e', 'axis_d', 'axis_c', 'axis_b']
        msg.position = [float(p) for p in positions]
        self._joint_pub.publish(msg)

    def _call_gripper(self, activate):
        if not self._gripper_client.service_is_ready():
            self.get_logger().warn('Gripper service not available')
            return
        req = SetBool.Request()
        req.data = activate
        self._gripper_client.call_async(req)

    def _elapsed(self):
        return (self.get_clock().now() - self._state_entry_time).nanoseconds / 1e9

    def _transition(self, new_state, duration=5.0):
        self.get_logger().info(f'State: {self._state} -> {new_state}')
        self._state = new_state
        self._state_entry_time = self.get_clock().now()
        self._state_duration = duration

    def _tick(self):
        elapsed = self._elapsed()

        if self._state == State.INIT:
            if elapsed > 2.0:
                self._transition(State.APPROACH_CUBE_A, 8.0)

        elif self._state == State.APPROACH_CUBE_A:
            self._send_thruster(surge=0.3)
            self._send_arm(self.ARM_STOWED)
            if elapsed > self._state_duration:
                self._transition(State.DESCEND_TO_CUBE_A, 6.0)

        elif self._state == State.DESCEND_TO_CUBE_A:
            self._send_thruster(heave=0.3)
            self._send_arm(self.ARM_REACH_DOWN)
            if elapsed > self._state_duration:
                self._transition(State.GRIP_CUBE_A, 3.0)

        elif self._state == State.GRIP_CUBE_A:
            self._send_thruster()
            self._call_gripper(True)
            if elapsed > self._state_duration:
                self._transition(State.LIFT_CUBE_A, 5.0)

        elif self._state == State.LIFT_CUBE_A:
            self._send_thruster(heave=-0.3)
            if elapsed > self._state_duration:
                self._transition(State.TRANSIT_TO_CUBE_B, 8.0)

        elif self._state == State.TRANSIT_TO_CUBE_B:
            self._send_thruster(surge=0.3)
            if elapsed > self._state_duration:
                self._transition(State.DESCEND_TO_CUBE_B, 6.0)

        elif self._state == State.DESCEND_TO_CUBE_B:
            self._send_thruster(heave=0.3)
            if elapsed > self._state_duration:
                self._transition(State.RELEASE_CUBE_A, 3.0)

        elif self._state == State.RELEASE_CUBE_A:
            self._send_thruster()
            self._call_gripper(False)
            if elapsed > self._state_duration:
                self._transition(State.ASCEND, 5.0)

        elif self._state == State.ASCEND:
            self._send_thruster(heave=-0.3)
            self._send_arm(self.ARM_STOWED)
            if elapsed > self._state_duration:
                self._transition(State.DONE)

        elif self._state == State.DONE:
            self._send_thruster()
            if elapsed < 1.0:
                self.get_logger().info('Demo complete!')


def main(args=None):
    rclpy.init(args=args)
    node = TestCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
