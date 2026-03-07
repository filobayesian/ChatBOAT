#!/usr/bin/env python3
"""Keyboard teleop for the Reach Alpha 5 arm joints.

Controls:
  q/a  - axis_e (yaw)      +/-
  w/s  - axis_d (shoulder)  +/-
  e/d  - axis_c (elbow)     +/-
  r/f  - axis_b (wrist)     +/-

  z    - zero all joints (stow)
  x    - increase step size
  c    - decrease step size
  ESC  - quit
"""
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

KEYS = {
    'q': (0, +1), 'a': (0, -1),   # axis_e
    'w': (1, +1), 's': (1, -1),   # axis_d
    'e': (2, +1), 'd': (2, -1),   # axis_c
    'r': (3, +1), 'f': (3, -1),   # axis_b
}

JOINT_NAMES = [
    'chatboat/axis_e',
    'chatboat/axis_d',
    'chatboat/axis_c',
    'chatboat/axis_b',
]

HELP = """
Arm Teleop - Keyboard Control
------------------------------
  q/a : axis_e (yaw)     +/-
  w/s : axis_d (shoulder) +/-
  e/d : axis_c (elbow)   +/-
  r/f : axis_b (wrist)   +/-

  z   : zero all joints
  x/c : increase/decrease step size
  ESC : quit

Step size: {step:.2f} rad
Positions: {pos}
"""


class ArmTeleop(Node):
    def __init__(self):
        super().__init__('arm_teleop')
        self._pub = self.create_publisher(JointState, '/chatboat/joint_commands', 10)
        self._timer = self.create_timer(0.1, self._publish)
        self._positions = [0.0, 0.0, 0.0, 0.0]
        self._step = 0.05

    def _publish(self):
        msg = JointState()
        msg.name = list(JOINT_NAMES)
        msg.position = [float(p) for p in self._positions]
        self._pub.publish(msg)

    def run(self):
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            self._print_status()
            while rclpy.ok():
                ch = sys.stdin.read(1)
                if ch == '\x1b':  # ESC
                    break
                elif ch in KEYS:
                    idx, direction = KEYS[ch]
                    self._positions[idx] += direction * self._step
                    self._print_status()
                elif ch == 'z':
                    self._positions = [0.0, 0.0, 0.0, 0.0]
                    self._print_status()
                elif ch == 'x':
                    self._step = min(0.5, self._step + 0.01)
                    self._print_status()
                elif ch == 'c':
                    self._step = max(0.01, self._step - 0.01)
                    self._print_status()
                rclpy.spin_once(self, timeout_sec=0)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def _print_status(self):
        pos_str = ', '.join(f'{p:+.2f}' for p in self._positions)
        # Clear screen and print
        sys.stdout.write('\033[2J\033[H')
        sys.stdout.write(HELP.format(step=self._step, pos=f'[{pos_str}]'))
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = ArmTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
