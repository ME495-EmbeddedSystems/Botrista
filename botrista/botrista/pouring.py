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
        self.path = None
        self.get_logger().warn("Pouring started")

        # Moveit Wrapper Object
        self.moveit = MoveItApi(self,
                                "panda_link0",
                                "panda_hand_tcp",
                                "panda_manipulator",
                                "joint_states")

        # Creating Services
        self.pour_client = self.create_service(Empty,
                                              "pour_kettle",
                                              self.pour_callback)
        self.execute = self.create_service(Empty,
                                           "execute_traj",
                                           self.execute_callback)

        # TODO: get april tag home position and use that
        self.home = Point(x=0.3069, y=0.0, z=0.487)
        self.homePose = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

    async def pour_callback(self, request, response):
        # TODO: Fill in
        waypoints = get_spiral_waypoints(self.home,
                                         self.homePose,
                                         100,
                                         0.0,
                                         0.01,
                                         4.0,
                                         flipStart=False)
        result = await self.moveit.create_cartesian_path(waypoints)
        self.path = result.trajectory
        return response

    async def execute_callback(self, request, response):
        # Temporary
        self.moveit.execute_trajectory(self.path)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Pouring()
    rclpy.spin(node)
    rclpy.shutdown()


def get_spiral_waypoints(start: Point,
                         ore: Quaternion,
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
                                            z=start.z),
                             orientation=ore))
    if flipStart:
        poseList.reverse()

    return poseList
