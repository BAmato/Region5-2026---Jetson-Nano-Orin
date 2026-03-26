#!/usr/bin/env python3
"""Temporary stub so launch files do not crash before the real detector exists."""
import rclpy
from rclpy.node import Node
class MaterialDetectorNode(Node):
    def __init__(self):
        super().__init__('material_detector_node')
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.6)
        self.get_logger().warn('material_detector_node is a stub. Replace with the real model/inference node later.')
def main(args=None):
    rclpy.init(args=args)
    node = MaterialDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
