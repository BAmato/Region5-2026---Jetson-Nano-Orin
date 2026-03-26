from setuptools import find_packages, setup
package_name = 'mining_mayhem'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/jetson_bringup.launch.py', 'launch/vision_stack_smoke_test.launch.py','launch/rviz_bringup.launch.py']),
        ('share/' + package_name + '/config', ['config/apriltag_config.yaml', 'config/front_camera_calib.yaml', 'config/led_camera_calib.yaml', 'config/ekf.yaml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='UTEP RAS', maintainer_email='placeholder@example.com',
    description='Mining Mayhem autonomy stack (refactor bundle)', license='Proprietary',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'apriltag_to_odom_node = mining_mayhem.apriltag_to_odom_node:main',
        'telemetry_decoder_node = mining_mayhem.telemetry_decoder_node:main',
        'start_led_detector_node = mining_mayhem.start_led_detector_node:main',
        'match_watchdog_node = mining_mayhem.match_watchdog_node:main',
        'material_detector_node = mining_mayhem.material_detector_node:main',
        'path_planner_node = mining_mayhem.path_planner_node:main',
        'serial_bridge_node = mining_mayhem.serial_bridge_node:main',
        'mission_state_node = mining_mayhem.mission_state_node:main',
    ]},
)
