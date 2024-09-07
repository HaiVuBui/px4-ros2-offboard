import rclpy
from rclpy.node import Node
import csv
from nav_msgs.msg import Odometry  # Use the appropriate message type for your application

class PositionVelocityLogger(Node):

    def __init__(self):
        super().__init__('position_velocity_logger')

        # Subscribe to the odometry topic (adjust topic name as needed)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',  # or '/vehicle_local_position' or another topic
            self.odom_callback,
            10
        )

        # Open a CSV file in write mode
        self.csv_file = open('position_velocity_log.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write the header row
        self.csv_writer.writerow(['Time', 'Position_X', 'Position_Y', 'Position_Z', 'Velocity_X', 'Velocity_Y', 'Velocity_Z'])

    def odom_callback(self, msg):
        # Extract time, position, and velocity data
        current_time = self.get_clock().now().to_msg().sec
        position = msg.pose.pose.position
        velocity = msg.twist.twist.linear

        # Log the data to the console (optional)
        self.get_logger().info(f"Time: {current_time} Position: ({position.x}, {position.y}, {position.z}) Velocity: ({velocity.x}, {velocity.y}, {velocity.z})")

        # Write the data to the CSV file
        self.csv_writer.writerow([current_time, position.x, position.y, position.z, velocity.x, velocity.y, velocity.z])

    def destroy_node(self):
        # Close the CSV file when the node is destroyed
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PositionVelocityLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
