#!/usr/bin/env python3
"""Keyboard teleop for the BlueROV2 Heavy vehicle.

Controls:
  Arrow Up/Down   - surge (forward / backward)
  W/S             - heave (ascend / descend)

  Space           - stop all thrusters
  +/-             - increase/decrease power
  Q               - quit
"""
import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

HELP = """
Vehicle Teleop - Keyboard Control
-----------------------------------
  Arrow Up   : surge forward
  Arrow Down : surge backward
  W          : ascend
  S          : descend

  Space      : stop all
  +/-        : adjust power
  R/F        : adjust roll compensation
  Q          : quit

Power: {power:.0f}   Roll comp: {roll:.2f}
Surge: {surge:+.0f}   Heave: {heave:+.0f}
"""

# Stonefish thrusters have max_setpoint=1000, quadratic thrust model
MAX_SETPOINT = 1000.0


class VehicleTeleop(Node):
    def __init__(self):
        super().__init__('vehicle_teleop')
        self._pub = self.create_publisher(
            Float64MultiArray, '/chatboat/thruster_commands', 10)
        self._timer = self.create_timer(0.1, self._publish)
        self._surge = 0.0
        self._heave = 0.0
        self._power = 10.0  # setpoint out of 1000
        self._roll_comp = 0.15  # roll compensation factor (tunable)

    def _publish(self):
        # BlueROV2 Heavy 8-thruster mixing
        surge = self._surge
        heave = self._heave

        # Horizontal thrusters at ±45° angles
        # FR and FL are inverted=true in Stonefish, so pre-negate
        t1 = -surge   # FrontRight (inverted=true, pre-negate)
        t2 = -surge   # FrontLeft  (inverted=true, pre-negate)
        t3 = -surge   # BackRight  (inverted=false)
        t4 = -surge   # BackLeft   (inverted=false)

        # Vertical thrusters with roll compensation
        # DFL and DBR are inverted=true in Stonefish, so pre-negate
        roll_comp = self._roll_comp * surge
        t5 =  heave - roll_comp    # DFR (inverted=false)
        t6 = -(heave + roll_comp)  # DFL (inverted=true, pre-negate)
        t7 = -(heave - roll_comp)  # DBR (inverted=true, pre-negate)
        t8 =  heave + roll_comp    # DBL (inverted=false)

        msg = Float64MultiArray()
        msg.data = [max(-MAX_SETPOINT, min(MAX_SETPOINT, t))
                     for t in [t1, t2, t3, t4, t5, t6, t7, t8]]
        self._pub.publish(msg)

    def _read_key(self):
        """Read a keypress, handling arrow key escape sequences."""
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            # Could be an arrow key sequence (ESC [ A/B/C/D) or bare ESC
            if select.select([sys.stdin], [], [], 0.05)[0]:
                ch2 = sys.stdin.read(1)
                if ch2 == '[' and select.select([sys.stdin], [], [], 0.05)[0]:
                    ch3 = sys.stdin.read(1)
                    if ch3 == 'A':
                        return 'UP'
                    elif ch3 == 'B':
                        return 'DOWN'
            return None  # ignore bare ESC / unknown sequences
        return ch

    def run(self):
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            self._print_status()
            while rclpy.ok():
                key = self._read_key()
                if key == 'q' or key == 'Q':
                    break
                elif key == 'UP':
                    self._surge = self._power
                    self._print_status()
                elif key == 'DOWN':
                    self._surge = -self._power
                    self._print_status()
                elif key in ('w', 'W'):
                    self._heave = -self._power  # negative = ascend
                    self._print_status()
                elif key in ('s', 'S'):
                    self._heave = self._power   # positive = descend
                    self._print_status()
                elif key == ' ':
                    self._surge = 0.0
                    self._heave = 0.0
                    self._print_status()
                elif key == '+' or key == '=':
                    self._power = min(MAX_SETPOINT, self._power + 5.0)
                    self._print_status()
                elif key == '-':
                    self._power = max(5.0, self._power - 5.0)
                    self._print_status()
                elif key in ('r', 'R'):
                    self._roll_comp += 0.05
                    self._print_status()
                elif key in ('f', 'F'):
                    self._roll_comp -= 0.05
                    self._print_status()
                rclpy.spin_once(self, timeout_sec=0)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def _print_status(self):
        sys.stdout.write('\033[2J\033[H')
        sys.stdout.write(HELP.format(
            power=self._power, roll=self._roll_comp,
            surge=self._surge, heave=self._heave))
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = VehicleTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
