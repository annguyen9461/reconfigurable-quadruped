#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Int32  # The topic message type
from quad_interfaces.action import Move   # The action type

from rclpy.qos import qos_profile_sensor_data
import time

class MoveActionClient(Node):
    def __init__(self):
        super().__init__('move_action_client')

        # Create an action client
        self._action_client = ActionClient(self, Move, 'move')

        # Subscribe to /num_bowling_pins topic
        self.sub_num_pins = self.create_subscription(
            Int32,
            '/num_bowling_pins',
            self.command_callback,
            qos_profile_sensor_data
        )

        self.current_command = None  # Track last sent command

    def command_callback(self, msg):
        """Callback function for the /move_command topic."""
        bowling_pin_count = msg.data
        
        self.get_logger().info(f"Received pin count: {bowling_pin_count}")

        # Determine new command based on pin count
        new_command = "turn" if bowling_pin_count < 2 else "stop"

        self.get_logger().info(f"New Command: {new_command}, Current Command: {self.current_command}")

        # Only send a goal if the command has changed
        if self.current_command != new_command:
            self.current_command = new_command
            self.send_goal(self.current_command)

    def send_goal(self, movement_type):
        """Send an action goal to the move action server."""
        goal_msg = Move.Goal()
        goal_msg.movement = movement_type

        self.get_logger().info(f"Sending goal: {movement_type}")

        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("⚠️ Action server unavailable, skipping goal")
            return
        send_goal_future = self._action_client.send_goal_async(goal_msg, self.goal_response_callback)
        send_goal_future.add_done_callback(self.result_callback)
        time.sleep(0.5)

    def goal_response_callback(self, future):
        """Callback function when the action server accepts or rejects the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("🚨 Goal rejected by the server!")
            return

        self.get_logger().info("✅ Goal accepted!")

        # Log that we're waiting for the result
        self.get_logger().info("⏳ Waiting for action result...")

        # Wait for result
        goal_handle.get_result_async().add_done_callback(self.result_callback)


    def result_callback(self, future):
        """Callback function when the action server returns the result."""
        result = future.result().get_result()
        if result.success:
            self.get_logger().info("Goal succeeded!")
        else:
            self.get_logger().warn("Goal failed!")


def main(args=None):
    rclpy.init(args=args)
    client_node = MoveActionClient()
    rclpy.spin(client_node)
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
