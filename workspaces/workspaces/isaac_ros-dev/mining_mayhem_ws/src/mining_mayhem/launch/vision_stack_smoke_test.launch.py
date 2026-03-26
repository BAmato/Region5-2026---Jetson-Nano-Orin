#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
FIELD_X = 2.4384
FIELD_Y = 1.2192
TAG_Z = 0.20
TAG5_X = FIELD_X / 2.0
TAG5_Y = FIELD_Y
TAG5_YAW = 3.14159
TAG6_X = FIELD_X / 2.0
TAG6_Y = 0.0
TAG6_YAW = 0.0
TAG7_X = FIELD_X
TAG7_Y = FIELD_Y / 2.0
TAG7_YAW = 1.5708
CAM_X = 0.10
CAM_Y = 0.00
CAM_Z = 0.25
CAM_PITCH = -0.175
CAM_YAW = 0.0
CAM_ROLL = 0.0
def generate_launch_description():
    pkg_share = get_package_share_directory('mining_mayhem')
    config_dir = os.path.join(pkg_share, 'config')
    apriltag_config = os.path.join(config_dir, 'apriltag_config.yaml')
    front_calib = os.path.join(config_dir, 'front_camera_calib.yaml')
    front_camera_node = Node(
        package='usb_cam', executable='usb_cam_node_exe', name='front_camera_driver', output='screen',
        parameters=[{'video_device': '/dev/video0','image_width': 640,'image_height': 480,'framerate': 30.0,
                     'pixel_format': 'yuyv2rgb','camera_name': 'usb_fallback_cam','camera_frame_id': 'camera_link',
                     'camera_info_url': f'file://{front_calib}'}],
        remappings=[('image_raw','/image'),('camera_info','/camera_info')],)
    apriltag_container = ComposableNodeContainer(
        name='apriltag_container', namespace='', package='rclcpp_components', executable='component_container_mt', output='screen',
        composable_node_descriptions=[ComposableNode(package='isaac_ros_apriltag', plugin='nvidia::isaac_ros::apriltag::AprilTagNode', name='apriltag',
            remappings=[('image','/image'),('camera_info','/camera_info'),('tag_detections','/apriltag/detections')], parameters=[{'family':'36h11','size':0.1524,'max_tags':8}])],)
    apriltag_to_odom_node = Node(package='mining_mayhem', executable='apriltag_to_odom_node', name='apriltag_to_odom_node', output='screen', parameters=[apriltag_config])
    telemetry_decoder_node = Node(package='mining_mayhem', executable='telemetry_decoder_node', name='telemetry_decoder_node', output='screen', parameters=[apriltag_config])
    tf_tag5 = Node(package='tf2_ros', executable='static_transform_publisher', name='tf_tag5', arguments=['--x', str(TAG5_X), '--y', str(TAG5_Y), '--z', str(TAG_Z), '--yaw', str(TAG5_YAW), '--pitch', '0.0', '--roll', '0.0', '--frame-id', 'map', '--child-frame-id', 'tag_5'])
    tf_tag6 = Node(package='tf2_ros', executable='static_transform_publisher', name='tf_tag6', arguments=['--x', str(TAG6_X), '--y', str(TAG6_Y), '--z', str(TAG_Z), '--yaw', str(TAG6_YAW), '--pitch', '0.0', '--roll', '0.0', '--frame-id', 'map', '--child-frame-id', 'tag_6'])
    tf_tag7 = Node(package='tf2_ros', executable='static_transform_publisher', name='tf_tag7', arguments=['--x', str(TAG7_X), '--y', str(TAG7_Y), '--z', str(TAG_Z), '--yaw', str(TAG7_YAW), '--pitch', '0.0', '--roll', '0.0', '--frame-id', 'map', '--child-frame-id', 'tag_7'])
    tf_front_camera = Node(package='tf2_ros', executable='static_transform_publisher', name='tf_front_camera', arguments=['--x', str(CAM_X), '--y', str(CAM_Y), '--z', str(CAM_Z), '--yaw', str(CAM_YAW), '--pitch', str(CAM_PITCH), '--roll', str(CAM_ROLL), '--frame-id', 'base_link', '--child-frame-id', 'camera_link'])
    return LaunchDescription([front_camera_node, apriltag_container, apriltag_to_odom_node, telemetry_decoder_node, tf_tag5, tf_tag6, tf_tag7, tf_front_camera])
