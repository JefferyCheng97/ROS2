import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
import cv2
from ament_index_python.packages import get_package_share_directory
import os
from cv_bridge import CvBridge
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

class FaceDetectClientNode(Node):
    def __init__(self):
        super().__init__('face_detect_client_node_param')
        self.bridge = CvBridge()
        self.default_image_path = os.path.join(get_package_share_directory('demo_python_service'), 'resource/test.jpg')
        self.get_logger().info("人脸识别客户端启动...")
        self.client = self.create_client(FaceDetector,'/face_detect_param')
        self.image = cv2.imread(self.default_image_path)


    def call_set_parameters(self, parameters):
        update_param = self.create_client(SetParameters, '/face_detect_node_param/set_parameters')

        while update_param.wait_for_service(timeout_sec = 1.0) is False:
            self.get_logger().info("等待参数更新服务端上线...")
        
        request = SetParameters.Request()
        request.parameters = parameters

        # 发送请求并等待处理完成
        future = update_param.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        return response

    """ 根据传入的的model, 构造Parametes, 然后调用call_set_parameters函数更新参数 """
    def update_detect_model(self, model='hog'):
        # 1. 创建Parameter对象
        param = Parameter()
        param.name = 'model'

        # 2. 创建ParameterValue对象并赋值
        param_value = ParameterValue()
        param_value.string_value = model
        param_value.type = ParameterType.PARAMETER_STRING
        param.value = param_value

        # 3. 调用更新参数的函数
        response = self.call_set_parameters([param])

        for result in response.results:
            if result.successful:
                self.get_logger().info(f"成功更新参数model为{model}")
            else:
                self.get_logger().info("参数更新失败")


    def send_request(self):
        # 1.判断服务端是否在线
        while self.client.wait_for_service(timeout_sec = 1.0) is False:
            self.get_logger().info("等待服务端上线...")

        # 2.构造request
        request = FaceDetector.Request()
        request.image = self.bridge.cv2_to_imgmsg(self.image)

        # 3.发送请求并等待处理完成
        future = self.client.call_async(request) # 需要等待服务端处理完成，才会把结果放到future中，否则没有数据

        def result_callback(result_future):
            response = result_future.result() # 获取响应
            self.get_logger().info(f"接受到响应，共检测到有{response.number}张人脸，耗时{response.use_time}s")
            
            # self.show_response(response) # 调用图片显示函数

        future.add_done_callback(result_callback)


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
    node.update_detect_model('hog')
    node.send_request()
    node.update_detect_model('cnn')
    node.send_request()
    rclpy.spin(node)
    rclpy.shutdown()