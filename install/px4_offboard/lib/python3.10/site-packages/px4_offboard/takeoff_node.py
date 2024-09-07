import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandTOL

class takeoff_node(Node):
    def __init__(self):
        super().__init__('takeoff_node')
        self.client = self.create_client(CommandTOL, 'mavros/cmd/takeoff')
        self.timer = self.create_timer(5.0, self.send_takeoff_command)  # Adjust time as needed

    def send_takeoff_command(self):
        if not self.client.service_is_ready():
            self.get_logger().info('Service not available, waiting...')
            return

        request = CommandTOL.Request()
        request.altitude = 10.0  # Desired altitude in meters
        request.latitude = 0.0
        request.longitude = 0.0
        request.min_pitch = 0.0

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Takeoff command sent!')
        else:
            self.get_logger().error('Failed to send takeoff command')

def main(args=None):
    rclpy.init(args=args)
    node = takeoff_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
