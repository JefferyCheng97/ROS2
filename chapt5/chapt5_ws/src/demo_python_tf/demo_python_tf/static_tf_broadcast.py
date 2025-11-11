import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster # 静态坐标发不起
from geometry_msgs.msg import TransformStamped # 消息接口
from tf_transformations import quaternion_from_euler # 欧拉角转四元数
import math # 角度转弧度

class StaticTFBroadcaster(Node):
    def __init__(self):
        super().__init__('static_tf_broadcaster') # 节点名称
        self.static_broadcaster_ = StaticTransformBroadcaster(self) # 创建静态坐标发布器
        self.publish_static_tf() # 发布静态坐标

        
    def publish_static_tf(self):
        """
        发布静态TF,从base_link到camera_link之间的坐标关系
        """
        transform = TransformStamped() # 创建坐标变换消息
        transform.header.stamp = self.get_clock().now().to_msg() # 时间戳
        transform.header.frame_id = 'base_link' # 父坐标系
        transform.child_frame_id = 'camera_link' # 子坐标系

        # 设置平移部分
        transform.transform.translation.x = 0.5
        transform.transform.translation.y = 0.3
        transform.transform.translation.z = 0.6

        # 设置旋转部分(欧拉角转四元数), 返回值的顺序是x, y, z, w
        q = quaternion_from_euler(math.radians(180), 0, 0)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]

        # 发布静态坐标变换
        self.static_broadcaster_.sendTransform(transform)
        self.get_logger().info(f'发布静态TF:{transform}')


def main():
    rclpy.init() # 初始化rclpy
    node = StaticTFBroadcaster() # 创建节点实例
    rclpy.spin(node) # 保持节点运行
    rclpy.shutdown() # 关闭rclpy