import rclpy
from rclpy.node import Node
import math
import numpy as np
import time

# Object Importing
from moveit_wrapper.moveitapi import MoveItApi
from geometry_msgs.msg import Point, Quaternion, Pose, Vector3
from rcl_interfaces.msg import ParameterDescriptor
from moveit_msgs.msg import RobotState
from std_srvs.srv import Empty


class Pouring(Node):
    def __init__(self):
        super().__init__(node_name="pouring")
        self.get_logger().warn("Pouring started")

        # Moveit Wrapper Object
        self.moveit = MoveItApi(self,
                                "panda_link0",
                                "panda_hand_tcp",
                                "panda_manipulator",
                                "panda_gripper/joint_states")

        # Creating Services
        self.pour_client = self.create_client(Empty,
                                              "pour_kettle",
                                              10,
                                              self.pour_callback)

        # TODO: get april tag home position and use that

    def pour_callback(self, request, response):
        # TODO: Fill in
        return response


def main(args=None):
    rclpy.init(args=args)
    Pouring = Pouring()
    rclpy.spin(Pouring)
    rclpy.shutdown()


def get_spiral_waypoints(start: Point,
                         numPoints: int,
                         a: float,
                         b: float,
                         loops: float,
                         flipStart: bool = False) -> list[Pose]:
    """
    Create a spiral path given parameters

    Arguments:
        start (geometry_msgs/Point) -- Starting position
        numPoints (int) -- number of points used to build the path
        a (float) -- distance of the starting point from the origin
        b (float) -- distance between turns of the spiral
        loops -- number of loops for the spiral to go through

    Keyword Arguments:
        flipStart (bool) -- Start at the end of the spiral instead of the center (default: {False})

    Returns:
        A list of waypoints

    """
    count = 0
    thTotal = loops*2*math.pi
    thStep = thTotal/numPoints

    # A list of tuples that contains r and theta
    polarCoords = []
    while count < numPoints:
        polarCoords.append(((count*thStep)*b+a, count*thStep))
        count += 1

    poseList = []
    # Convert polar coordinates into cartesian and add in start offset
    for coord in polarCoords:
        x = coord[0]*math.cos(coord[1]) + start.x
        y = coord[0]*math.sin(coord[1]) + start.y
        poseList.append(Pose(position=Point(x=x,
                                            y=y,
                                            z=start.z)))

    if flipStart:
        return poseList.reverse

    return poseList
