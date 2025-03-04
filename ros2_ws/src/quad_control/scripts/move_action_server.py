#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from quad_interfaces.action import Move
from std_msgs.msg import String

class MoveActionServer(Node):

    def __init__(self):
        super().__init__('move_action_server')
        self._action_server = ActionServer(
            self,
            Move,
            'move',
            execute_callback=self.execute_callback,
        )

        # Subscription to /move_command
        self.sub_move_command = self.create_subscription(
            String,
            '/move_command',
            self.execute_callback,
            10
        )

        # Publisher for feedback
        self.pub_feedback = self.create_publisher(String, '/move_feedback', 10)
        
        self.current_command = "stop" # Default movement state
        self.get_logger().info("MoveActionServer is ready.")

    def execute_callback(self, goal_handle):
        """Execute the action, updating movement until the command changes."""
        movement_type = goal_handle.request.movement
        self.get_logger().info(f"Executing movement: {movement_type}")
        
        # Populate result message
        result = Move.Result()
        return result



def main(args=None):
    rclpy.init(args=args)

    node = MoveActionServer()

    rclpy.spin(node)


if __name__ == '__main__':
    main()