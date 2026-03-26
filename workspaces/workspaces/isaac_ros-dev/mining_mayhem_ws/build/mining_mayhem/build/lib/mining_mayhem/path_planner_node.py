#!/usr/bin/env python3
"""Temporary stub so mission bringup can evolve without crashing launch."""
import rclpy
from rclpy.node import Node
class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')
        self.declare_parameter('max_linear_speed', 0.4)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('waypoint_tolerance', 0.03)
        self.declare_parameter('heading_tolerance', 0.05)
        self.get_logger().warn('path_planner_node is currently a stub. Keep mission_state simple until planner is implemented.')
def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
