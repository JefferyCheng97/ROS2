import rclpy
from rclpy.node import Node

def main():
    rclpy.init() 
    node = Node('python_node')
    node.get_logger().info('你好 python 节点！')
    node.get_logger().warn('测试 python 节点！')
    rclpy.spin(node)
    rclpy.shutdown()
