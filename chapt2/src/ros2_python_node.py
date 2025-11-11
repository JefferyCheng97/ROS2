import rclpy
from rclpy.node import Node

def main():
    rclpy.init() 
    node = Node('python_node')
    node.get_logger().info('python 节点！')
    node.get_logger().warn('python 节点！')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()