from launch import LaunchDescription
<<<<<<< HEAD
<<<<<<< HEAD
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, GroupAction
=======
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
>>>>>>> 5d7f16f (Added launch file that starts the realsense node for both)
=======
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, GroupAction
>>>>>>> 460e438 (Added localizer node that localizes camera to the robot)
from launch.substitutions import PathJoinSubstitution


# this is a launch file launches both the d435i and the d405, the april
# tag node, and the camera_localizer node

def generate_launch_description():
    return LaunchDescription([
<<<<<<< HEAD
<<<<<<< HEAD
        GroupAction(actions=[
            SetRemap(src='/d435i/color/image_raw',
                     dst='/image_raw'),
            SetRemap(src='/d435i/color/camera_info', dst='/camera_info'),
=======
        GroupAction(actions=[
            # SetRemap(src='/d435i/color/image_raw/compressed',
            #          dst='/image_rect/compressed'),
            # SetRemap(src='/d435i/color/camera_info', dst='/camera_info'),
>>>>>>> 460e438 (Added localizer node that localizes camera to the robot)
            IncludeLaunchDescription(
                PathJoinSubstitution([
                    FindPackageShare("realsense2_camera"),
                    "launch",
                    "rs_launch.py"
                ]),
                launch_arguments={
                    'camera_name': 'd435i',
                    'device_type': 'd435i',
<<<<<<< HEAD
                    'rgb_camera.profile': '1920x1080x6',
                    'enable_depth': 'false',
=======
>>>>>>> 460e438 (Added localizer node that localizes camera to the robot)
                }.items(),
            ),
        ]),
        # IncludeLaunchDescription(
        #     PathJoinSubstitution([
        #         FindPackageShare("realsense2_camera"),
        #         "launch",
        #         "rs_launch.py"
        #     ]),
        #     launch_arguments={
        #         'camera_name': 'd405',
        #         'device_type': 'd405',
        #         'pointcloud.enable': 'true',
        #     }.items(),
        # ),
<<<<<<< HEAD
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("image_proc"),
                "launch",
                "image_proc.launch.py"
            ])
        ),
=======
        # IncludeLaunchDescription(
        #     PathJoinSubstitution([
        #         FindPackageShare("image_proc"),
        #         "launch",
        #         "image_proc.launch.py"
        #     ])
        # ),
>>>>>>> 460e438 (Added localizer node that localizes camera to the robot)
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            ros_arguments=[
                "--params-file",
                PathJoinSubstitution([
                    FindPackageShare("botrista"),
                    "config",
                    "tag.yaml"
                ])
            ],
<<<<<<< HEAD
            # remappings={
            #     'image_rect': 'image_raw',
            # }.items()
        ),
        Node(
            package="botrista",
            executable="camera_localizer",
        )
=======
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"),
                "launch",
                "rs_launch.py"
            ]),
            launch_arguments={
                'camera_name': 'd435i',
                'device_type': 'd435i',
                'pointcloud.enable': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"),
                "launch",
                "rs_launch.py"
            ]),
            launch_arguments={
                'camera_name': 'd405',
                'device_type': 'd405',
                'pointcloud.enable': 'true',
            }.items(),
        ),
>>>>>>> 5d7f16f (Added launch file that starts the realsense node for both)
=======
            remappings={
                'image_rect': 'd435i/color/image_raw',
                'image_rect/compressed': 'd435i/color/image_raw/compressed',
                'camera_info': 'd435i/color/camera_info',
            }.items()
        ),
        Node(
            package="botrista",
            executable="camera_localizer",
        )
>>>>>>> 460e438 (Added localizer node that localizes camera to the robot)
    ])
