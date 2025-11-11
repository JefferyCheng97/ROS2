from demo_python_pkg.person_node import PersonNode

class WriterNode(PersonNode):
    def __init__(self, name:str, age:int, book: str) -> None:
        print(" WriterNode 构造函数被调用")
        super().__init__(name, age)
        self.book = book
        
def main():
    node = WriterNode("苏轼", 30, "论经济发展")
    node.eat("鱼香肉丝")