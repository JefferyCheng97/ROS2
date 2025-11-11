import rclpy
from rclpy.node import Node

class PersonNode(Node):
    def __init__(self, node_name: str, name_value: str, age_value: int) -> None:
        print("PersonNode 构造函数被调用")
        super().__init__(node_name)
        self.name = name_value
        self.age = age_value

    def eat(self, food_name: str):
        """吃东西"""
        # print(f"{self.name}, {self.age}岁, 爱吃{food_name}")
        self.get_logger().info(f"{self.name}, {self.age}岁, 爱吃{food_name}")   

def main():
    rclpy.init()
    node = PersonNode("libai", "李白", 20)
    node.eat("月饼")
    rclpy.spin(node)
    rclpy.shutdown()