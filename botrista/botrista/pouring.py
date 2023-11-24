import rclpy
from rclpy.node import Node
import math
import numpy as np
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

# Object Importing
from moveit_wrapper.moveitapi import MoveItApi
from geometry_msgs.msg import Vector3, Point, Quaternion, Pose, TransformStamped
from shape_msgs.msg import SolidPrimitive
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer
from botrista_interfaces.action import PourAction
from std_srvs.srv import Empty


class Pouring(Node):
    def __init__(self):
        super().__init__(node_name="pouring")
        self.path = None
        self.cb = ReentrantCallbackGroup()
        self.get_logger().warn("Pouring started")

        # Creating Parameters
        self.declare_parameter("pour_begin_offset", 0.05,
                               ParameterDescriptor(
                                   description="Z distance to move to before pouring"))
        self.offset = self.get_parameter("pour_begin_offset")\
            .get_parameter_value().double_value

        # Creating tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_parent_frame = "panda_link0"

        # Moveit Wrapper Object
        self.moveit = MoveItApi(self,
                                "panda_link0",
                                "panda_hand_tcp",
                                "panda_manipulator",
                                "joint_states",
                                "panda")

        # Creating action server
        self._action_server = ActionServer(self,
                                           PourAction,
                                           'pour_action',
                                           self.pour_callback,
                                           callback_group=self.cb)
        
        self.test = self.create_service(Empty,
                                        "test_attach",
                                        self.test,
                                        callback_group=self.cb)

    async def pour_callback(self, goal_handle):
        # TODO: Fill in
        result = PourAction.Result()
        feedback = PourAction.Feedback()
        req = goal_handle.request

        # Attempt to get point from frame
        try:
            tf = self.tf_buffer.lookup_transform(
                self.tf_parent_frame,
                req.pour_frame,
                rclpy.time.Time())
            startVec = tf.transform.translation
            startPoint = Point(x=startVec.x, y=startVec.y, z=startVec.z)
            startOre = tf.transform.rotation
        except TransformException:
            feedback.stage = f"Failed to get transform to {req.pour_frame}"
            goal_handle.publish_feedback(feedback)
            result.status = False
            return result

        # Calculating path
        feedback.stage = "Calculating path"
        goal_handle.publish_feedback(feedback)
        waypoints = get_spiral_waypoints(startPoint,
                                               req.tilt_ang,
                                               req.num_points,
                                               req.spiral_radius,
                                               req.num_loops,
                                               req.start_outside)
        standoff_point = startPoint
        standoff_point.z += self.offset
        standoffPose = Pose(position=standoff_point,
                            orientation=startOre)
        waypoints.append(standoffPose)
        waypoints.insert(0, standoffPose)

        # Planning
        feedback.stage = "Planning path"
        goal_handle.publish_feedback(feedback)
        planned_traj = await self.moveit.create_cartesian_path(waypoints)
        
        # Executing path
        feedback.stage = "Executing path"
        goal_handle.publish_feedback(feedback)
        self.moveit.execute_trajectory(planned_traj.trajectory)

        goal_handle.succeed()
        result.status = True
        return result

    async def test(self, request, response):
        # arr = []
        # link = TransformStamped()
        # link.header.frame_id = "handle"
        # link.header.stamp = self.get_clock().now().to_msg()
        # link.child_frame_id = "panda_link0"
        # arr.append(link)

        # link1 = TransformStamped()
        # link1.header.frame_id = "handle2"
        # link1.header.stamp = self.get_clock().now().to_msg()
        # link1.child_frame_id = "panda_link0"
        # link1.transform.translation=Vector3(x=1.0)
        # link1.transform.rotation=Quaternion(x=1.0)
        # arr.append(link1)
        # result = await self.moveit.attachObject(arr)
        # print(result)
    

        pose = Pose(position=Point(x=0.0, y=0.0, z=0.1),
                    orientation=Quaternion(x=1.0))
        primitive = SolidPrimitive(
            type=SolidPrimitive.BOX,
            dimensions=[0.02, 0.02, 0.02]
        )
        self.moveit.createAttachObject("box", [pose], [primitive])
        self.moveit.attachObjectToEE("box")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Pouring()
    rclpy.spin(node)
    rclpy.shutdown()


def get_spiral_waypoints(startPoint: Point,
                         startOre: Quaternion,
                         numPoints: int,
                         maxRadius: float,
                         loops: float,
                         flipStart: bool = False) -> (list[Pose], float):
    """
    Create a spiral path given parameters

    Arguments:
        startPoint (geometry_msgs/Point) -- Starting point
        startOre (geometry_msgs/Quaternion) -- Starting Orientation
        numPoints (int) -- number of points used to build the path
        maxRadius (float) -- distance from end of spiral to origin in cm
        loops (float) -- number of loops for the spiral to go through

    Keyword Arguments:
        flipStart (bool) -- Start at the end of the spiral instead of the center (default: {False})

    Returns:
        A list of waypoints

    """
    count = 0
    thTotal = loops*2*math.pi
    thStep = thTotal/numPoints
    b = maxRadius/2/math.pi/loops

    # Create poses for each point along the spiral
    poseList = []
    while count < numPoints:
        # Calculating new point
        th = count*thStep
        r = th*b
        x = r*math.cos(th) + startPoint.x
        y = r*math.sin(th) + startPoint.y
        poseList.append(Pose(position=Point(x=x,
                                            y=y,
                                            z=startPoint.z),
                             orientation=startOre))

        count += 1

    if flipStart:
        poseList.reverse

    return poseList
