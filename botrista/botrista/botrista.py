from enum import Enum, auto
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
import math
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

# Import types
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Vector3, Pose
from botrista_interfaces.action import PourAction
from std_srvs.srv import Empty


class State(Enum):
    pass


class Botrista(Node):

    def __init__(self):
        super().__init__("botrista")
        self.cb_group = ReentrantCallbackGroup()

        # Action Clients
        self.pour_action_client = ActionClient(self, PourAction, 'pour_action', callback_group=self.cb_group)

        # Service Clients
        self.kettle_grab = self.create_client(Empty, "grab", callback_group=self.cb_group)
        self.kettle_release = self.create_client(Empty, "place", callback_group=self.cb_group)


        # Timeouts
        if not self.pour_action_client.wait_for_server(timeout_sec=1.0):
            raise RuntimeError(
                'Timeout waiting for "Pour" action to become available')
        if not self.kettle_grab.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(
                'Timeout waiting for "grab" service to become available')
        if not self.kettle_release.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(
                'Timeout waiting for "release" service to become available')




def main(args=None):
    rclpy.init(args=args)
    botrista = Botrista()
    rclpy.spin(botrista)
    rclpy.shutdown()