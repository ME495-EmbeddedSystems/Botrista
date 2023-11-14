import rclpy
from rclpy.node import Node
import math
import numpy as np
import time
<<<<<<< HEAD
from rclpy.callback_groups import ReentrantCallbackGroup
=======
>>>>>>> 6daa95b (Worked on pouring node:)

# Object Importing
from moveit_wrapper.moveitapi import MoveItApi
from geometry_msgs.msg import Point, Quaternion, Pose, Vector3
from rcl_interfaces.msg import ParameterDescriptor
from moveit_msgs.msg import RobotState
from std_srvs.srv import Empty


class Pouring(Node):
    def __init__(self):
        super().__init__(node_name="pouring")
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 3b543a4 (Worked on pouring node:)
        self.path = None
        self.get_logger().warn("Pouring started")
        self.cb = ReentrantCallbackGroup()
=======
        self.get_logger().warn("Pouring started")
>>>>>>> 6daa95b (Worked on pouring node:)

        # Moveit Wrapper Object
        self.moveit = MoveItApi(self,
                                "panda_link0",
                                "panda_hand_tcp",
                                "panda_manipulator",
<<<<<<< HEAD
<<<<<<< HEAD
                                "joint_states")

        # Creating Services
        self.pour_client = self.create_service(Empty,
                                               "pour_kettle",
                                               self.pour_callback,
                                               callback_group=self.cb)
        self.execute = self.create_service(Empty,
                                           "execute_traj",
                                           self.execute_callback,
                                           callback_group=self.cb)

        # TODO: get april tag home position and use that
        self.home = Point(x=0.3069, y=0.0, z=0.487)
        self.homePose = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

    async def pour_callback(self, request, response):
        # TODO: Fill in
        self.get_logger().warn("started cb")
        waypoints = get_spiral_waypoints(self.home,
                                         1000,
                                         0.1,
                                         10.0,
                                         endStart=False)
        result = await self.moveit.create_cartesian_path(waypoints)
        self.get_logger().warn("finished await")
        self.path = result.trajectory
        return response

    async def execute_callback(self, request, response):
        # Temporary
        self.moveit.execute_trajectory(self.path)
=======
                                "panda_gripper/joint_states")
=======
                                "joint_states")
>>>>>>> 3b543a4 (Worked on pouring node:)

        # Creating Services
        self.pour_client = self.create_service(Empty,
                                              "pour_kettle",
                                              self.pour_callback)
        self.execute = self.create_service(Empty,
                                           "execute_traj",
                                           self.execute_callback)

        # TODO: get april tag home position and use that
        self.home = Point(x=0.217, y=0.096, z=0.622)
        self.homePose = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    async def pour_callback(self, request, response):
        # TODO: Fill in
<<<<<<< HEAD
>>>>>>> 6daa95b (Worked on pouring node:)
=======
        waypoints = get_spiral_waypoints(self.home,
                                         self.homePose,
                                         10,
                                         0.0,
                                         0.01,
                                         1.0,
                                         flipStart=True)
        result = await self.moveit.create_cartesian_path(waypoints)
        self.path = result.trajectory
        return response

    async def execute_callback(self, request, response):
        # Temporary
        self.moveit.execute_trajectory(self.path)
>>>>>>> 3b543a4 (Worked on pouring node:)
        return response


def main(args=None):
    rclpy.init(args=args)
<<<<<<< HEAD
<<<<<<< HEAD
    node = Pouring()
    rclpy.spin(node)
    rclpy.shutdown()


def get_spiral_waypoints(startPoint: Point,
                         numPoints: int,
                         maxRadius: float,
                         loops: float,
                         endStart: bool = False) -> list[Pose]:
=======
    Pouring = Pouring()
    rclpy.spin(Pouring)
=======
    node = Pouring()
    rclpy.spin(node)
>>>>>>> 3b543a4 (Worked on pouring node:)
    rclpy.shutdown()


def get_spiral_waypoints(start: Point,
                         ore: Quaternion,
                         numPoints: int,
                         a: float,
                         b: float,
                         loops: float,
                         flipStart: bool = False) -> list[Pose]:
>>>>>>> 6daa95b (Worked on pouring node:)
    """
    Create a spiral path given parameters

    Arguments:
<<<<<<< HEAD
        startPoint (geometry_msgs/Point) -- Starting point
        numPoints (int) -- number of points used to build the path
        maxRadius (float) -- distance from end of spiral to origin in cm
        loops -- number of loops for the spiral to go through

    Keyword Arguments:
        endStart (bool) -- Start at the end of the spiral instead of the center (default: {False})
=======
        start (geometry_msgs/Point) -- Starting position
        numPoints (int) -- number of points used to build the path
        a (float) -- distance of the starting point from the origin
        b (float) -- distance between turns of the spiral
        loops -- number of loops for the spiral to go through

    Keyword Arguments:
        flipStart (bool) -- Start at the end of the spiral instead of the center (default: {False})
>>>>>>> 6daa95b (Worked on pouring node:)

    Returns:
        A list of waypoints

    """
    count = 0
    thTotal = loops*2*math.pi
    thStep = thTotal/numPoints
<<<<<<< HEAD
    b = maxRadius/2/math.pi/loops

    # Create poses for each point along the spiral
    poseList = []
    while count < numPoints:
        th = count*thStep
        r = th*b
        x = r*math.cos(th) + startPoint.x
        y = r*math.sin(th) + startPoint.y
        # print("(", x, ",", y, ")")
        poseList.append(Pose(position=Point(x=x,
                                            y=y,
                                            z=startPoint.z),
                             orientation=Quaternion(x=1.0,
                                                    y=0.,
                                                    z=0.,
                                                    w=0.)))

        count += 1

    if endStart:
        poseList.reverse()
=======

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
<<<<<<< HEAD
        return poseList.reverse
>>>>>>> 6daa95b (Worked on pouring node:)
=======
        poseList.reverse()
>>>>>>> 3b543a4 (Worked on pouring node:)

    return poseList
