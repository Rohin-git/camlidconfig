import open3d as o3d
import rclpy
from rclpy.node import Node

class MyNewNode(Node):
    def __init__(self):
        super().__init__('my_new_node')

        # Initialize any attributes or variables
        self.some_variable = 0

    def some_method(self):
        # Define any methods
        print("This is a method of MyNewNode")

def main(args=None):
    rclpy.init(args=args)
    my_new_node = MyNewNode()
    rclpy.spin(my_new_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
