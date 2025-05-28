from irobot_create_msgs.srv import EStop
import rclpy
from rclpy.node import Node
from .pycreate2.create2api import Create2

class RoombaService(Node):
    def __init__(self):
        super().__init__('roomba_service')
        self.get_logger().info('Roomba Service Node has been started.')
        self.roomba = Create2('/dev/ttyUSB0')
        self.srv = self.create_service(EStop, 'estop', self.estop_callback) 

    def estop_callback(self, request, response):
        response.success = True
        response.message = 'E-Stop command received'
        self.get_logger().info('Incoming request\ne_stop_on: %s\n e_stop_off: %s' % (request.e_stop_on))
        return response

def main(args=None):
    rclpy.init(args=args)
    roomba_service = RoombaService()
    try:
        rclpy.spin(roomba_service)
    except KeyboardInterrupt:
        pass
    finally:
        roomba_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()