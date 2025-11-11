import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer # 动态坐标监听器
from tf_transformations import euler_from_quaternion # 四元数转欧拉角
import math # 角度转弧度

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener') # 节点名称
        self.buffer_ = Buffer() # 创建坐标缓存对象
        self.listener = TransformListener(self.buffer_, self) # 创建坐标监听器
        self.timer = self.create_timer(1.0, self.get_transform) # 创建定时器,每1秒监听一次TF


    def get_transform(self):
        """
        定时获取坐标，实时查询坐标关系
        """
        try:
            result = self.buffer_.lookup_transform('base_link', 
                'bottle_link', rclpy.time.Time(seconds = 0.0), rclpy.time.Duration(seconds = 1.0))
            transform = result.transform
            self.get_logger().info(f'平移:{transform.translation}')
            self.get_logger().info(f'旋转:{transform.rotation}')
            rotation_euler = euler_from_quaternion([
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w
            ])
            self.get_logger().info(f'旋转RPY:{rotation_euler}')

        except Exception as e:
            self.get_logger().warn(f'获取坐标变换失败:{e}')


def main():
    rclpy.init() # 初始化rclpy
    node = TFListener() # 创建节点实例
    rclpy.spin(node) # 保持节点运行
    rclpy.shutdown() # 关闭rclpy 