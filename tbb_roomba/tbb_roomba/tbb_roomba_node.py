import rclpy
from rclpy.node import Node
from pycreate2 import Create2

class TbbRoombaNode(Node):
    def __init__(self):
        super().__init__('tbb_roomba_node')
        self.get_logger().info('TBB Roomba Node has been started.')
        self.roomba = Create2('/dev/ttyUSB0')

def main(args=None):
    rclpy.init(args=args)
    tbb_roomba_node = TbbRoombaNode()
    
    rclpy.spin(tbb_roomba_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

