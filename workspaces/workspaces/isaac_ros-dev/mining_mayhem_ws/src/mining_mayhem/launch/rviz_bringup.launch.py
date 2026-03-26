#!/usr/bin/env python3
# ===========================================================================
# rviz_bringup.launch.py
# Launches RViz with a pre-configured view for Mining Mayhem debugging.
#
# Shows: robot pose, field, AprilTags, camera feed, odometry path, match state
#
# Usage (from host terminal with ROS_DOMAIN_ID=42):
#   ros2 launch mining_mayhem rviz_bringup.launch.py
#
# Assumes jetson_bringup.launch.py is already running in another terminal.
# ===========================================================================

import os
import tempfile
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_rviz_config() -> str:
    config = """
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded: ~
      Splitter Ratio: 0.5
    Tree Height: 600
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Nav Goal1
    Name: Tool Properties
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views

Visualization Manager:
  Class: ""
  Displays:
    # -----------------------------------------------------------------------
    # Grid — field floor reference
    # -----------------------------------------------------------------------
    - Class: rviz_default_plugins/Grid
      Name: Field Grid
      Enabled: true
      Cell Size: 0.3048
      Color: 160; 160; 164
      Line Style:
        Line Width: 0.01
        Value: Lines
      Normal Cell Count: 0
      Offset:
        X: 1.22
        Y: 0.61
        Z: 0
      Plane: XY
      Plane Cell Count: 8
      Reference Frame: map

    # -----------------------------------------------------------------------
    # TF — all coordinate frames
    # -----------------------------------------------------------------------
    - Class: rviz_default_plugins/TF
      Name: TF Frames
      Enabled: true
      Frame Timeout: 5
      Frames:
        All Enabled: true
      Marker Scale: 0.3
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Tree:
        map:
          odom:
            base_link:
              camera_link: {}
              led_camera_link: {}

    # -----------------------------------------------------------------------
    # Robot odometry path (wheel)
    # -----------------------------------------------------------------------
    - Class: rviz_default_plugins/Odometry
      Name: Wheel Odometry
      Enabled: true
      Topic:
        Value: /odom/wheel
        Depth: 5
        QoS Profile: sensor_data
      Keep: 200
      Position Tolerance: 0.01
      Angle Tolerance: 0.05
      Shape:
        Alpha: 0.5
        Axes Length: 0.1
        Axes Radius: 0.01
        Value: Arrow
      Color: 255; 165; 0
      Head Length: 0.05
      Head Radius: 0.03
      Shaft Length: 0.1
      Shaft Radius: 0.01

    # -----------------------------------------------------------------------
    # EKF filtered odometry
    # -----------------------------------------------------------------------
    - Class: rviz_default_plugins/Odometry
      Name: EKF Filtered
      Enabled: true
      Topic:
        Value: /odom/filtered
        Depth: 5
        QoS Profile: sensor_data
      Keep: 200
      Position Tolerance: 0.005
      Angle Tolerance: 0.02
      Color: 0; 255; 0
      Head Length: 0.07
      Head Radius: 0.04
      Shaft Length: 0.15
      Shaft Radius: 0.015

    # -----------------------------------------------------------------------
    # Camera feed
    # -----------------------------------------------------------------------
    - Class: rviz_default_plugins/Image
      Name: Front Camera
      Enabled: true
      Topic:
        Value: /image
        Depth: 1
        QoS Profile: sensor_data
      Normalize Range: true
      Transport Hint: raw

    # -----------------------------------------------------------------------
    # AprilTag detections marker array
    # -----------------------------------------------------------------------
    - Class: rviz_default_plugins/MarkerArray
      Name: AprilTag Detections
      Enabled: true
      Topic:
        Value: /apriltag/detections_viz
        Depth: 5

    # -----------------------------------------------------------------------
    # IMU
    # -----------------------------------------------------------------------
    - Class: rviz_default_plugins/Imu
      Name: IMU
      Enabled: true
      Topic:
        Value: /imu/data
        Depth: 5
        QoS Profile: sensor_data
      Box Scale: 0.08
      Show Acceleration: false
      Show Axes: true
      Show Trail: false

  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/SetInitialPose
      Topic:
        Value: /initialpose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Value: /clicked_point
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/TopDownOrtho
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Scale: 130
      Target Frame: map
      Value: TopDownOrtho (rviz_default_plugins)
      X: -0.3
      Y: -0.2
    Saved: ~
"""
    config_path = os.path.join(tempfile.gettempdir(), 'mining_mayhem.rviz')
    with open(config_path, 'w') as f:
        f.write(config)
    return config_path


def generate_launch_description():
    rviz_config = generate_rviz_config()

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([rviz_node])
