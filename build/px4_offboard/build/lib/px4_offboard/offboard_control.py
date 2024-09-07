#!/usr/bin/env python

import rclpy
import math
import statistics
import numpy as np
import csv  # Import the CSV module
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleOdometry, VehicleCommand  # Import VehicleOdometry message type


class OffboardControl(Node):

    def __init__(self):
        super().__init__('offboard_control_node')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.time = 0
        self.com=list()
        self.min=0
        self.max=0
        self.mean=0
        # Subscriptions
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        self.odometry_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.vehicle_odometry_callback,
            qos_profile)

        # Publishers
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.dt = timer_period
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Initialize state variables
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.current_position = [float('nan'), float('nan'), float('nan')]  # Placeholder for current position
        self.current_velocity = [0.0, 0.0, 0.0]  # Placeholder for current velocity
        self.previous_position = [float('nan'), float('nan'), float('nan')]  # Placeholder for previous position
        self.previous_velocity = [0.0, 0.0, 0.0]  # Placeholder for previous velocity

        # Open CSV file for writing
        self.csv_file = open('data/trajectory_data.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # Write the header
        self.csv_writer.writerow([                               
                                  'previous_position_x', 'previous_position_y', 'previous_position_z',
                                  'previous_velocity_x', 'previous_velocity_y', 'previous_velocity_z',
                                   'current_position_x', 'current_position_y', 'current_position_z'])

    def vehicle_status_callback(self, msg):
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.publisher_offboard_mode.publish(msg)

    def vehicle_odometry_callback(self, msg):
        # Update previous position and velocity from current state
        self.previous_position = self.current_position.copy()
        self.previous_velocity = self.current_velocity.copy()

        # Update current position and velocity from odometry
        self.current_position[0] = msg.position[0]
        self.current_position[1] = msg.position[1]
        self.current_position[2] = msg.position[2]
        self.current_velocity[0] = msg.velocity[0]
        self.current_velocity[1] = msg.velocity[1]
        self.current_velocity[2] = msg.velocity[2]

    def cmdloop_callback(self):
        # Publish offboard control modes
        self.publish_offboard_control_heartbeat_signal()

        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and
                self.arming_state == VehicleStatus.ARMING_STATE_ARMED):

            trajectory_msg = TrajectorySetpoint()
            trajectory = self.trajectory(self.time)

            trajectory_msg.velocity[0] = trajectory[0]
            trajectory_msg.velocity[1] = trajectory[1]
            trajectory_msg.velocity[2] = trajectory[2]
            trajectory_msg.position[0] = trajectory[3]
            trajectory_msg.position[1] = trajectory[4]
            trajectory_msg.position[2] = trajectory[5]

            self.time += 0.02

            self.publisher_trajectory.publish(trajectory_msg)
            self.get_logger().info(f"Published trajectory: {trajectory_msg}")
            if not self.time >10:
                d_position=self.path_function(self.time)
                self.com.append(math.sqrt((self.current_position[0]-d_position[0])**2
                                          +(self.current_position[1]-d_position[1])**2
                                          +(self.current_position[2]-d_position[2])**2))

                self.max=max(self.com)
                self.min=min(self.com)
                self.mean=statistics.mean(self.com)

            # Record data to CSV
            timestamp = Clock().now().nanoseconds / 1e9
            self.csv_writer.writerow([
                self.time,
                self.previous_position[0],
                self.previous_position[1],
                self.previous_position[2],
                self.previous_velocity[0],
                self.previous_velocity[1],
                self.previous_velocity[2],
                self.current_position[0],
                self.current_position[1],
                self.current_position[2],
                self.max,
                self.mean,
                self.min
            ])
  

            if self.time>100:
                self.destroy_node()
        else:
            self.get_logger().info("Waiting for arming and offboard mode activation.")
            self.engage_offboard_mode()
            self.arm()

    def path_function(self,t):
        if t < 10:
            x = 0
            y = 0
            z = 0
        else:
            x=math.sin(t-10)
            y=t-10
            z=0
        return x,y,z 

    def trajectory(self, t):
        if t < 10:
            x = 0
            y = 0
            z = 0
            x_p = 0
            y_p = 0
            z_p = -5
        else:
            # x = math.cos(t-10)
            # y = 1
            # z = 0
            x=0
            y=0
            z=0
            x_p = math.sin(t-10)
            y_p = t-10
            z_p = -5
            # x_p=float("nan")
            # y_p=float("nan")
            # z_p=float("nan")
        return (x, y, z, x_p, y_p, z_p)

    def destroy_node(self):
        super().destroy_node()
        # Close CSV file when node is destroyed
        self.csv_file.close()


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
