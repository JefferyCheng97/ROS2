import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
import cv2
from ament_index_python.packages import get_package_share_directory
import os
from cv_bridge import CvBridge

class FaceDetectClientNode(Node):
    def __init__(self):
        super().__init__('face_detect_client_node')
        self.bridge = CvBridge()
        self.default_image_path = os.path.join(get_package_share_directory('demo_python_service'), 'resource/test.jpg')
        self.get_logger().info("人脸识别客户端启动...")
        self.client = self.create_client(FaceDetector,'/face_detect')
        self.image = cv2.imread(self.default_image_path)

    def send_request(self):
        # 1.判断服务端是否在线
        while self.client.wait_for_service(timeout_sec = 1.0) is False:
            self.get_logger().info("等待服务端上线...")

        # 2.构造request
        request = FaceDetector.Request()
        request.image = self.bridge.cv2_to_imgmsg(self.image)

        # 3.发送请求并等待处理完成
        future = self.client.call_async(request)# 需要等待服务端处理完成，才会把结果放到future中，否则没有数据
        # # 如果没有处理完成，休眠等待，但会堵塞线程，使当前线程无法接受来自服务端的返回
        # while not future.done():
        #     time.sleep(1.0)
        # 以异步的方式向服务端发送服务请求，返回的future是一个类似于占位符的东西，在服务端处理完请求并返回结果后存放对应的数据
        # 会在不阻塞线程正常运行的情况下，持续等待服务端返回响应，直到future对应的请求有了结果
        rclpy.spin_until_future_complete(self, future)
        response = future.result()# 获取响应
        self.get_logger().info(f"接受到响应，共检测到有{response.number}张人脸，耗时{response.use_time}s")

        # 调用图片显示函数
        self.show_response(response)


    def show_response(self, response):
        for i in range(response.number):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom[i]
            left = response.left[i]
            cv2.rectangle(self.image, (left, top), (right,bottom), (255,0,0), 4)
        cv2.imshow('Face detect result', self.image)
        self.get_logger().info("图片已显示...")
        cv2.waitKey(0) # 也会堵塞，导致spin无法正常运行


def main(args = None):
    rclpy.init(args = args)
    node = FaceDetectClientNode()
    node.send_request()
    rclpy.spin(node)
    rclpy.shutdown()