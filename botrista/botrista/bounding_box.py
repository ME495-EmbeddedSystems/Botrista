import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, TransformStamped, Transform, Vector3
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
import itertools
from typing import List
import cv2 as cv2
import math
import tf2_geometry_msgs
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import Header


class BoundingBoxNode(Node):

    def __init__(self):
        super().__init__("bounding_box_node")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self, 10)

        # create static bounding box for testing
        self.bounding_box = create_bounding_box(
            Vector3(
                x=0.3
            ),
            Vector3(
                x=0.2,
                y=0.2,
                z=0.2
            ))

        self.camera_info_sub = self.create_subscription(CameraInfo,
                                                        'camera_info',
                                                        self.camera_info_callback,
                                                        qos_profile=10)

        self.image_callback = self.create_subscription(Image,
                                                       'image_rect',
                                                       self.image_callback,
                                                       qos_profile=10)

        self.bounding_box_pub = self.create_publisher(Image,
                                                      'bounding_image',
                                                      qos_profile=10)

        self.pinhole_camera_model = None
        self.camera_info = None
        self.pinhole_camera_model = PinholeCameraModel()

        self.cv_bridge = CvBridge()

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def image_callback(self, msg):
        if self.camera_info is None:
            # return if the camera model isn't defined yet
            return

        # create camera info
        self.pinhole_camera_model.fromCameraInfo(self.camera_info)

        # convert image to cv2
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

        # project each bounding box point to image
        # cv_image_points = [self.pinhole_camera_model.project3dToPixel(
        #     (pose.position.x,
        #      pose.position.y,
        #      pose.position.z))
        #     for pose in self.bounding_box]

        self.get_logger().warn(f"looking up transform")
        try:
            robot_camera_tf = self.tf_buffer.lookup_transform(
                "d435i_color_optical_frame", "panda_link0", Time(seconds=0.0))
        except Exception:
            return

        bounding_box = [tf2_geometry_msgs.do_transform_pose(
            pose, robot_camera_tf) for pose in self.bounding_box]

        bounding_box_points = [(pose.position.x, pose.position.y, pose.position.z)
                               for pose in bounding_box]

        image_points = [[round(e) for e in self.pinhole_camera_model.project3dToPixel(p)]
                        for p in bounding_box_points]

        for p, ip in zip(bounding_box_points, image_points):
            self.get_logger().warn(f"{p} {ip}")

        # draw each point on image
        for image_point in image_points:
            self.get_logger().warn(f"drawing point {image_point}")
            cv2.circle(cv_image, image_point, 3, (255, 0, 0), 2)

        out_image = self.cv_bridge.cv2_to_imgmsg(cv_image, 'rgb8', Header(
            frame_id="d435i_color_optical_frame",
            stamp=self.get_clock().now().to_msg()
        ))

        self.bounding_box_pub.publish(out_image)


def create_bounding_box(loc: Vector3, dim: Vector3) -> List[Pose]:
    """
    Creates a bounding box.

    Arguments:
        + loc (geometry_msgs/msg/Vector3) - the location of the bounding box
        + dim (geometry_msgs/msg/Vector3) - the x, y, and z dimensions of the bpx

    Returns:
    -------
        A list of poses for each corner of the bounding box.

    """
    poses = []
    for (a, b, c) in itertools.product([0, 1], repeat=3):
        poses.append(
            Pose(
                position=Point(
                    x=loc.x + dim.x*a,
                    y=loc.y + dim.y*b,
                    z=loc.z + dim.z*c
                )
            )
        )
    return poses


def main(args=None):
    rclpy.init(args=args)
    node = BoundingBoxNode()
    rclpy.spin(node)
    rclpy.shutdown()
