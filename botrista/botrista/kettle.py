import rclpy
from rclpy.node import Node
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener
from moveit_wrapper.moveitapi import MoveItApi
from moveit_wrapper.grasp_planner import GraspPlan, GraspPlanner
from geometry_msgs.msg import Pose, Point, Quaternion
from std_srvs.srv import Empty
from rclpy.callback_groups import ReentrantCallbackGroup
from franka_msgs.action import Grasp
from rclpy.time import Time
from rclpy.action import ActionClient
from botrista_interfaces.action import PourAction


class Kettle(Node):

    def __init__(self):
        super().__init__("kettle")
        self.cb_group = ReentrantCallbackGroup
        self.kettle_hand_to_spout = 0.05 #TODO: input correct distance

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.moveit_api = MoveItApi(
            self, "panda_link0", "panda_hand_tcp", "panda_manipulator", "/franka/joint_states")
        self.grasp_planner = GraspPlanner(
            self.moveit_api, "panda_gripper/grasp")

        self.grab_srv = self.create_service(
            Empty, "grab", self.grab, callback_group=self.cb_group)

        self.release_srv = self.create_service(
            Empty, "place", self.place, callback_group=self.cb_group)

        # measured poses
        self.approach_pose = Pose(
            position=Point(x=0.006, y=0.05, z=0.20),
            orientation=Quaternion(x=0.88, y=-0.035, z=0.47, w=0.01)
        )
        self.grasp_pose = Pose(
            position=Point(x=0.097, y=0.043, z=0.15),
            orientation=Quaternion(x=0.88, y=-0.035, z=0.47, w=0.01)
        )
        self.retreat_pose = Pose(
            position=Point(x=0.097, y=0.043, z=0.25),
            orientation=Quaternion(x=0.88, y=-0.035, z=0.47, w=0.01)
        )

        # Pouring
        self.pour_action_client = ActionClient(
            self, PourAction, 'pour_action', callback_group=self.cb_group)
        if not self.pour_action_client.wait_for_server(timeout_sec=1.0):
            raise RuntimeError(
                'Timeout waiting for "Pour" action to become available')

        # Pour action variables
        pour_req = PourAction.Goal(spiral_radius=0.05,
                                   num_loops=100,
                                   num_points=100,
                                   pour_frame="test",
                                   tilt_ang=Quaternion(x=0.611741304397583,
                                                       y=0.6160799264907837,
                                                       z=0.3496427536010742,
                                                       w=-0.3520910441875458),
                                   y_offset=self.kettle_hand_to_spout)

    async def grab(self, request, response):
        """
        Grabs the kettle from its stand.
        """
        tf = self.buffer.lookup_transform(
            "panda_link0", "filtered_kettle_tag", Time())

        approach_pose = tf2_geometry_msgs.do_transform_pose(
            self.approach_pose, tf)
        grasp_pose = tf2_geometry_msgs.do_transform_pose(self.grasp_pose, tf)
        retreat_pose = tf2_geometry_msgs.do_transform_pose(
            self.retreat_pose, tf)

        grasp_plan = GraspPlan(
            approach_pose=approach_pose,
            grasp_pose=grasp_pose,
            grasp_command=Grasp.Goal(
                width=0.02,
                force=50.0,
                speed=0.05,
            ),
            retreat_pose=retreat_pose,
        )

        await self.grasp_planner.execute_grasp_plan(grasp_plan)

        return response

    async def place(self, request, response):
        """
        Places the kettle on its stand.
        """
        tf = self.buffer.lookup_transform(
            "panda_link0", "filtered_kettle_tag", Time())

        # play the grasp plan backwards to place the kettle
        approach_pose = tf2_geometry_msgs.do_transform_pose(
            self.retreat_pose, tf)

        hover_pose = Pose(
            position=Point(
                x=0.09,
                y=0.043,
                z=0.18),
            orientation=Quaternion(x=0.88, y=-0.035, z=0.47, w=0.01)
        )
        grasp_pose = tf2_geometry_msgs.do_transform_pose(hover_pose, tf)
        retreat_pose = tf2_geometry_msgs.do_transform_pose(
            self.approach_pose, tf)

        grasp_plan = GraspPlan(
            approach_pose=approach_pose,
            grasp_pose=grasp_pose,
            grasp_command=Grasp.Goal(
                width=0.04,  # open the gripper wider to release the kettle
                force=50.0,
                speed=0.2,
            ),
            retreat_pose=retreat_pose,
        )

        await self.grasp_planner.execute_grasp_plan(grasp_plan)

        return response


def kettle_entry(args=None):
    rclpy.init(args=args)
    kettle = Kettle()
    rclpy.spin(kettle)
    rclpy.shutdown()
