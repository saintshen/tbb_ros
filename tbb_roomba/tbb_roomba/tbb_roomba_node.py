import rclpy
from rclpy.node import Node

class TbbRoombaNode(Node):
    def __init__(self):
        super().__init__('tbb_roomba_node')

def main(args=None):
    rclpy.init(args=args)
    tbb_roomba_node = TbbRoombaNode()
    rclpy.spin(tbb_roomba_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

