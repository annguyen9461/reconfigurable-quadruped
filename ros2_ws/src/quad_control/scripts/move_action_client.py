#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Int32  # The topic message type
from quad_interfaces.action import Move   # The action type

from action_msgs.msg import GoalStatus

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
            10
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

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    client_node = MoveActionClient()
    rclpy.spin(client_node)
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
