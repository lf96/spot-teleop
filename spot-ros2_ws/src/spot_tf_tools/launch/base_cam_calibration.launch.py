from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1) AprilTag detector (ajuste os tópicos da ZED)
    config_dir = os.path.join(
        get_package_share_directory('spot_tf_tools'),
        'config',
        'apriltag_params.yaml'
    )
    apriltag = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        remappings=[
            ('image_rect', '/zed/zed_node/rgb/image_rect_color'),
            ('camera_info','/zed/zed_node/rgb/camera_info'),
        ],
        parameters=[config_dir]
    )

    # 2) T_base_tag (se o tag está rígido na base; ajuste XYZ/quat)
    #base_to_tag = Node(
    #    package='tf2_ros', executable='static_transform_publisher', name='base_to_tag',
    #    arguments=['0.01831','-0.11701','0.02973','-0.7532633','0.011205','0.0004334','0.6576235','body','apriltag_0']
    # )

    # 3) Solver: publica T_base_cam assim que os dois acima existirem
    solver = Node(
        package='spot_tf_tools', executable='solve_base_cam', name='solve_base_cam',
        parameters=[{
            'base_frame': 'base',
            'cam_frame':  'zed_left_camera_optical_frame',
            'tag_frame':  'apriltag_0',
            'recompute':  True
        }],
        arguments=['--ros-args', '--log-level', 'WARN']
    )
    # PROVISORIO PARA TESTAR TF FIXO
    # solver = Node(
    #    package='tf2_ros', executable='static_transform_publisher', name='base_to_cam',
    #    arguments=['0.0757','-0.9503','0.776','0.868','-0.133','0.201','-0.434','base','zed_left_camera_optical_frame']
    # )
    # 4) Monitor para alteracao significativa de TF base -> cam
    tf_monitor = Node(
        package="spot_tf_tools",
        executable="tf_calib_monitor",
        name="tf_calib_monitor",
        parameters=[{
            "base_frame": "base",
            "cam_frame": "zed_left_camera_optical_frame",
            "pos_threshold_m": 0.05,
            "ang_threshold_deg": 3.0,
            "check_period_s": 5.0,
        }],
    )
    # 5) Segmentador já usando o TF correto
    segmenter_config = os.path.join(
        get_package_share_directory('spot_tf_tools'),
        'config',
        'robot_segmenter_params.yaml'
    )

    segmenter = Node(
        package='spot_tf_tools',
        executable='segmenter_supervisor',
        name='robot_segmenter',
        parameters=[{
            "segmenter_cmd":(
                "/home/spot-teleop/spot-ros2_ws/install/isaac_ros_cumotion/lib/isaac_ros_cumotion/robot_segmenter_node "            "--ros-args "
                "--ros-args "
                "-r __node:=cumotion_robot_segmenter "
                "--params-file " + segmenter_config
                )
        }],
        output="screen",
    )

    return LaunchDescription([apriltag, solver, tf_monitor, segmenter])

