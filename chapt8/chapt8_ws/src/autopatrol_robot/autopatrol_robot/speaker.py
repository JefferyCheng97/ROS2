import rclpy
from rclpy.node import Node
from autopatrol_interfaces.srv import SpeechText
import espeakng


class Speaker(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.speech_service = self.create_service(SpeechText, 'speech_text', self.speach_text_callback)
        self.speaker_ = espeakng.Speaker()
        self.speaker_.voice = 'zh'
        self.get_logger().info("Speaker节点已启动，等待语音合成请求...")


    def speach_text_callback(self, request, response):
        self.get_logger().info(f"正在准备朗读: {request.text}")
        self.speaker_.say(request.text)
        self.speaker_.wait()
        response.result = True

        return response


def main():
    rclpy.init()
    speaker_node = Speaker('speaker')
    rclpy.spin(speaker_node)
    rclpy.shutdown()