import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory
import os
from cv_bridge import CvBridge # 用于在ROS图像消息格式和OpenCV图像格式之间进行转换的工具类
import time

class FaceDetectNode(Node):
    def __init__(self):
        super().__init__('face_detect_node')
        self.bridge = CvBridge()
        # 建了一个服务，服务的类型是之前导入的FaceDetector，服务名称为/face_detect
        self.service_ = self.create_service(FaceDetector, '/face_detect', self.detect_face_callback)
        self.get_logger().info("执行create_service")
        # 把参数作为类属性
        self.number_of_times_to_upsample = 1
        self.model = 'hog'
        self.default_image_path = os.path.join(get_package_share_directory('demo_python_service'), 'resource/default.jpg')
        self.get_logger().info("人脸识别服务启动...")
        

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