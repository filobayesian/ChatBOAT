import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import Bool


class GripperService(Node):
    """Logical gripper activate/deactivate service.

    In Stonefish, the magnetic gripper works via material proximity
    (PermanentMagnet + Ferromagnetic). This node provides a logical
    service interface for the control layer.
    """

    def __init__(self):
        super().__init__('gripper_service')
        self._active = False

        self._srv = self.create_service(
            SetBool, '/girona500/gripper/activate', self._handle_gripper)

        self._state_pub = self.create_publisher(
            Bool, '/girona500/gripper/state', 10)

        self._timer = self.create_timer(0.1, self._publish_state)

        self.get_logger().info('Gripper service ready')

    def _handle_gripper(self, request, response):
        self._active = request.data
        action = 'activated' if self._active else 'deactivated'
        response.success = True
        response.message = f'Gripper {action}'
        self.get_logger().info(f'Gripper {action}')
        return response

    def _publish_state(self):
        msg = Bool()
        msg.data = self._active
        self._state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GripperService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
