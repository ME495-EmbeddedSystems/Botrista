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
<<<<<<< HEAD
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
=======
                                               self.pour_callback)
>>>>>>> a470504 (Worked on pouring node:)
        self.execute = self.create_service(Empty,
                                           "execute_traj",
                                           self.execute_callback)

        # TODO: get april tag home position and use that
        self.home = Point(x=0.3069, y=0.0, z=0.487)
        self.homePose = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

    async def pour_callback(self, request, response):
        # TODO: Fill in
<<<<<<< HEAD
>>>>>>> 6daa95b (Worked on pouring node:)
=======
        waypoints = get_spiral_waypoints(self.home,
                                         100,
                                         0.01,
                                         4.0,
                                         flipStart=False)
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
<<<<<<< HEAD
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
=======
>>>>>>> a470504 (Worked on pouring node:)
                         numPoints: int,
                         maxRadius: float,
                         loops: float,
<<<<<<< HEAD
                         flipStart: bool = False) -> list[Pose]:
>>>>>>> 6daa95b (Worked on pouring node:)
=======
                         endStart: bool = False) -> list[Pose]:
>>>>>>> a470504 (Worked on pouring node:)
    """
    Create a spiral path given parameters

    Arguments:
<<<<<<< HEAD
<<<<<<< HEAD
        startPoint (geometry_msgs/Point) -- Starting point
        numPoints (int) -- number of points used to build the path
        maxRadius (float) -- distance from end of spiral to origin in cm
        loops -- number of loops for the spiral to go through

    Keyword Arguments:
        endStart (bool) -- Start at the end of the spiral instead of the center (default: {False})
=======
        start (geometry_msgs/Point) -- Starting position
=======
        startPoint (geometry_msgs/Point) -- Starting point
>>>>>>> a470504 (Worked on pouring node:)
        numPoints (int) -- number of points used to build the path
        maxRadius (float) -- distance from end of spiral to origin in cm
        loops -- number of loops for the spiral to go through

    Keyword Arguments:
<<<<<<< HEAD
        flipStart (bool) -- Start at the end of the spiral instead of the center (default: {False})
>>>>>>> 6daa95b (Worked on pouring node:)
=======
        endStart (bool) -- Start at the end of the spiral instead of the center (default: {False})
>>>>>>> a470504 (Worked on pouring node:)

    Returns:
        A list of waypoints

    """
    count = 0
    thTotal = loops*2*math.pi
    thStep = thTotal/numPoints
<<<<<<< HEAD
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
=======
    b = maxRadius/2/math.pi/loops
>>>>>>> a470504 (Worked on pouring node:)

    # Create poses for each point along the spiral
    poseList = []
    while count < numPoints:
        th = count*thStep
        r = th*b
        x = r*math.cos(th) + startPoint.x
        y = r*math.sin(th) + startPoint.y
        print("(", x, ",", y, ")")
        poseList.append(Pose(position=Point(x=x,
                                            y=y,
<<<<<<< HEAD
                                            z=start.z),
                             orientation=ore))
    if flipStart:
<<<<<<< HEAD
        return poseList.reverse
>>>>>>> 6daa95b (Worked on pouring node:)
=======
=======
                                            z=startPoint.z),
                             orientation=Quaternion()))

        count += 1

    if endStart:
>>>>>>> a470504 (Worked on pouring node:)
        poseList.reverse()
>>>>>>> 3b543a4 (Worked on pouring node:)

    return poseList
