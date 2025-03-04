#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Int32  # The topic message type
from quad_interfaces.action import Move   # The action type
from quad_interfaces.msg import SetConfig

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

        self.pub_config = self.create_publisher(SetConfig, '/set_config', 10)
        
        self.current_command = None  # Track last sent command
        self.action_in_progress = False  # Prevent sending multiple goals

    def command_callback(self, msg):
        """Decides action based on the number of bowling pins detected."""
        bowling_pin_count = msg.data
        self.get_logger().info(f"Received pin count: {bowling_pin_count}")

        if bowling_pin_count >= 2:  # Found a pin, stop turning
            self.stop_moving()
            return
        else:
            self.keep_turning()

    def send_goal(self, movement_type):
        """Send an action goal to the move action server."""
        goal_msg = Move.Goal()
        goal_msg.movement = movement_type

        self.get_logger().info(f"Sending goal: {movement_type}")

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)

    def stop_moving(self):
        """Stop turning and transition to Home1 configuration."""
        self.get_logger().info("Pin detected! Stopping turn and transitioning to home1.")
        self.send_goal("stop")

        # Publish to `/set_config` to stop moving
        config_msg = SetConfig()
        config_msg.config_id = 1
        self.get_logger().info("Publishing transition to home1.")
        self.pub_config.publish(config_msg)
    
    def keep_turning(self):
        """Keep turning and until detect enough bowling pins."""
        self.get_logger().info("Not enough pins yet! Keep turning.")
        self.send_goal("turn")

        # Publish to `/set_config` to stop moving
        config_msg = SetConfig()
        config_msg.config_id = 5
        self.get_logger().info("Publishing congfig 5 to turn.")
        self.pub_config.publish(config_msg)

def main(args=None):
    rclpy.init(args=args)
    client_node = MoveActionClient()
    rclpy.spin(client_node)
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
