import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory
import os
from cv_bridge import CvBridge # 用于在ROS图像消息格式和OpenCV图像格式之间进行转换的工具类
import time
from rcl_interfaces.msg import SetParametersResult # 用于参数更新回调函数的返回值类型


class FaceDetectNode(Node):
    def __init__(self):
        super().__init__('face_detect_node_param')
        self.bridge = CvBridge()
        # 建了一个服务，服务的类型是之前导入的FaceDetector，服务名称为/face_detect_param
        self.service_ = self.create_service(FaceDetector, '/face_detect_param', self.detect_face_callback)
        self.get_logger().info("执行create_service") 
        
        # 使用父类的declare_parameter和get_parameter方法声明和获取参数，并设置默认值
        # 方便用户在启动节点时通过参数覆盖默认值
        self.declare_parameter('number_of_times_to_upsample', 1)
        self.declare_parameter('model', 'hog')
        self.number_of_times_to_upsample = self.get_parameter('number_of_times_to_upsample').value
        self.model = self.get_parameter('model').value

        self.default_image_path = os.path.join(get_package_share_directory('demo_python_service'), 'resource/default.jpg')
        self.get_logger().info("人脸识别服务启动...")
        self.add_on_set_parameters_callback(self.parameter_callback)


    def parameter_callback(self, params):   
        for param in params:
            self.get_logger().info(f"收到参数更新请求：{param.name} = {param.value}")

            if param.name == 'number_of_times_to_upsample':
                self.number_of_times_to_upsample = param.value
                self.get_logger().info(f"参数number_of_times_to_upsample更新为：{self.number_of_times_to_upsample}")
            elif param.name == 'model':
                self.model = param.value
                self.get_logger().info(f"参数model更新为：{self.model}")

        return SetParametersResult(successful=True)


    def detect_face_callback(self, request, response):
        if request.image.data:
            cv_image = self.bridge.imgmsg_to_cv2(request.image)
        else:
            # 使用cv2加载图片
            cv_image = cv2.imread(self.default_image_path)
            self.get_logger().info("传入图片为空，使用默认图片...")

        # 计算耗时
        start_time = time.time()
        self.get_logger().info("加载完成图片，开始识别...")
        face_locations = face_recognition.face_locations(cv_image, number_of_times_to_upsample = 1, model = 'hog')
        response.use_time = time.time() - start_time
        response.number = len(face_locations)

        for top,right,bottom,left in face_locations:
            response.top.append(top)
            response.right.append(right)
            response.bottom.append(bottom)
            response.left.append(left)

        return response
    

def main(args = None):
    rclpy.init(args = args)
    node = FaceDetectNode()
    rclpy.spin(node)
    rclpy.shutdown()