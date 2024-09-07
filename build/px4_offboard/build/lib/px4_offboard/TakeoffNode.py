import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from mavros_msgs.msg import CommandTOL
from mavros_msgs.srv import CommandTOL as CommandTOLService

class TakeoffNode(Node):

    def __init__(self):
        super().__init__('takeoff_node')
        self.client = self.create_client(CommandTOLService, '/mavros/cmd/takeoff')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.request = CommandTOL.Request()
        self.request.altitude = 10  # Altitude in meters
        self.request.latitude = 0
        self.request.longitude = 0
        self.request.min_pitch = 0

        self.takeoff()

    def takeoff(self):
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Takeoff command sent successfully')
        else:
            self.get_logger().error('Failed to send takeoff command')

def main(args=None):
    rclpy.init(args=args)
    node = TakeoffNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
