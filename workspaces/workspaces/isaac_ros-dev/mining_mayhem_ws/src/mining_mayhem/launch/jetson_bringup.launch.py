#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
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
LED_CAM_X = -0.10
LED_CAM_Y = 0.00
LED_CAM_Z = 0.15
LED_CAM_PITCH = 0.10
LED_CAM_YAW = 3.14159
LED_CAM_ROLL = 0.0
def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    enable_serial_bridge_arg = DeclareLaunchArgument('enable_serial_bridge', default_value='true')
    enable_mission_state_arg = DeclareLaunchArgument('enable_mission_state', default_value='true')
    enable_path_planner_arg = DeclareLaunchArgument('enable_path_planner', default_value='true')
    enable_material_detector_arg = DeclareLaunchArgument('enable_material_detector', default_value='false')
    enable_led_camera_arg = DeclareLaunchArgument('enable_led_camera', default_value='false')
    pkg_share = get_package_share_directory('mining_mayhem')
    config_dir = os.path.join(pkg_share, 'config')
    ekf_config = os.path.join(config_dir, 'ekf.yaml')
    apriltag_config = os.path.join(config_dir, 'apriltag_config.yaml')
    front_calib = os.path.join(config_dir, 'front_camera_calib.yaml')
    led_calib = os.path.join(config_dir, 'led_camera_calib.yaml')
    front_camera_node = Node(package='usb_cam', executable='usb_cam_node_exe', name='front_camera_driver', output='screen', parameters=[{'video_device':'/dev/video0','image_width':640,'image_height':480,'framerate':30.0,'pixel_format':'yuyv2rgb','camera_name':'usb_fallback_cam','camera_frame_id':'camera_link','camera_info_url':f'file://{front_calib}'}], remappings=[('image_raw','/image'),('camera_info','/camera_info')])
    led_camera_node = Node(package='usb_cam', executable='usb_cam_node_exe', name='led_camera_driver', output='screen', parameters=[{'video_device':'/dev/video1','image_width':640,'image_height':480,'framerate':30.0,'pixel_format':'yuyv2rgb','camera_name':'led_camera','camera_frame_id':'led_camera_link','camera_info_url':f'file://{led_calib}'}], remappings=[('image_raw','/led_cam/image_raw'),('camera_info','/led_cam/camera_info')], condition=IfCondition(LaunchConfiguration('enable_led_camera')))
    apriltag_container = ComposableNodeContainer(name='apriltag_container', namespace='', package='rclcpp_components', executable='component_container_mt', output='screen', composable_node_descriptions=[ComposableNode(package='isaac_ros_apriltag', plugin='nvidia::isaac_ros::apriltag::AprilTagNode', name='apriltag', remappings=[('image','/image'),('camera_info','/camera_info'),('tag_detections','/apriltag/detections')], parameters=[{'family':'36h11','size':0.1524,'max_tags':8}])])
    apriltag_to_odom_node = Node(package='mining_mayhem', executable='apriltag_to_odom_node', name='apriltag_to_odom_node', output='screen', parameters=[apriltag_config])
    telemetry_decoder_node = Node(package='mining_mayhem', executable='telemetry_decoder_node', name='telemetry_decoder_node', output='screen', parameters=[apriltag_config])
    ekf_node = Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen', parameters=[ekf_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}], remappings=[('odometry/filtered','/odom/filtered')])
    start_led_detector_node = Node(package='mining_mayhem', executable='start_led_detector_node', name='start_led_detector_node', output='screen', parameters=[apriltag_config])
    match_watchdog_node = Node(package='mining_mayhem', executable='match_watchdog_node', name='match_watchdog_node', output='screen', parameters=[apriltag_config])
    serial_bridge_node = Node(package='mining_mayhem', executable='serial_bridge_node', name='serial_bridge_node', output='screen', parameters=[{'serial_port':'/dev/ttyACM0','baud_rate':115200,'loop_rate_hz':50.0}], condition=IfCondition(LaunchConfiguration('enable_serial_bridge')))
    mission_state_node = Node(package='mining_mayhem', executable='mission_state_node', name='mission_state_node', output='screen', parameters=[{'landing_x':0.15,'landing_y':0.15,'beacon_mast_x':0.05,'beacon_mast_y':0.6096,'field_length_m':FIELD_X,'field_width_m':FIELD_Y,'intake_width_m':0.20}], condition=IfCondition(LaunchConfiguration('enable_mission_state')))
    path_planner_node = Node(package='mining_mayhem', executable='path_planner_node', name='path_planner_node', output='screen', parameters=[{'max_linear_speed':0.4,'max_angular_speed':1.5,'waypoint_tolerance':0.03,'heading_tolerance':0.05}], condition=IfCondition(LaunchConfiguration('enable_path_planner')))
    material_detector_node = Node(package='mining_mayhem', executable='material_detector_node', name='material_detector_node', output='screen', parameters=[{'model_path':os.path.join(config_dir,'material_model.engine'),'confidence_threshold':0.6}], condition=IfCondition(LaunchConfiguration('enable_material_detector')))
    tf_tag5 = Node(package='tf2_ros', executable='static_transform_publisher', name='tf_tag5', arguments=['--x', str(TAG5_X), '--y', str(TAG5_Y), '--z', str(TAG_Z), '--yaw', str(TAG5_YAW), '--pitch', '0.0', '--roll', '0.0', '--frame-id', 'map', '--child-frame-id', 'tag_5'])
    tf_tag6 = Node(package='tf2_ros', executable='static_transform_publisher', name='tf_tag6', arguments=['--x', str(TAG6_X), '--y', str(TAG6_Y), '--z', str(TAG_Z), '--yaw', str(TAG6_YAW), '--pitch', '0.0', '--roll', '0.0', '--frame-id', 'map', '--child-frame-id', 'tag_6'])
    tf_tag7 = Node(package='tf2_ros', executable='static_transform_publisher', name='tf_tag7', arguments=['--x', str(TAG7_X), '--y', str(TAG7_Y), '--z', str(TAG_Z), '--yaw', str(TAG7_YAW), '--pitch', '0.0', '--roll', '0.0', '--frame-id', 'map', '--child-frame-id', 'tag_7'])
    tf_front_camera = Node(package='tf2_ros', executable='static_transform_publisher', name='tf_front_camera', arguments=['--x', str(CAM_X), '--y', str(CAM_Y), '--z', str(CAM_Z), '--yaw', str(CAM_YAW), '--pitch', str(CAM_PITCH), '--roll', str(CAM_ROLL), '--frame-id', 'base_link', '--child-frame-id', 'camera_link'])
    tf_led_camera = Node(package='tf2_ros', executable='static_transform_publisher', name='tf_led_camera', arguments=['--x', str(LED_CAM_X), '--y', str(LED_CAM_Y), '--z', str(LED_CAM_Z), '--yaw', str(LED_CAM_YAW), '--pitch', str(LED_CAM_PITCH), '--roll', str(LED_CAM_ROLL), '--frame-id', 'base_link', '--child-frame-id', 'led_camera_link'])
    return LaunchDescription([use_sim_time_arg, enable_serial_bridge_arg, enable_mission_state_arg, enable_path_planner_arg, enable_material_detector_arg, enable_led_camera_arg, front_camera_node, led_camera_node, apriltag_container, apriltag_to_odom_node, telemetry_decoder_node, ekf_node, start_led_detector_node, match_watchdog_node, serial_bridge_node, mission_state_node, path_planner_node, material_detector_node, tf_tag5, tf_tag6, tf_tag7, tf_front_camera, tf_led_camera])
