import rclpy
from rclpy.node import Node
from time import sleep
from std_srvs.srv import Empty
from botrista_interfaces.srv import DelayTime
from rclpy.callback_groups import ReentrantCallbackGroup


class Delay_Node(Node):
    def __init__(self):
        super().__init__("delay_node")
        self.delay_timer = self.create_service(
            DelayTime, "delay", self.delay_callback, callback_group=ReentrantCallbackGroup())

    def delay_callback(self, request, response):
        sleep(request.time)
        return response


def delay_entry(args=None):
    rclpy.init(args=args)
    node = Delay_Node()
    rclpy.spin(node)
    rclpy.shutdown()
