#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, CancelResponse
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
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Subscription to /move_command
        self.sub_move_command = self.create_subscription(
            String,
            '/move_command',
            self.move_command_callback,
            10
        )

        # Publisher for feedback
        self.pub_feedback = self.create_publisher(String, '/move_feedback', 10)
        
        self.current_command = "stop" # Default movement state
        self.get_logger().info("MoveActionServer is ready.")

    def goal_callback(self, goal_request):
        """Accept or reject a goal."""
        if goal_request.movement not in ["turn", "stop", "hcir", "cirh"]:
            self.get_logger().warn("Invalid movement request!")
            return rclpy.action.GoalResponse.REJECT

        self.get_logger().info(f"Received movement request: {goal_request.movement}")
        return rclpy.action.GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle goal cancellation."""
        self.get_logger().info("Goal canceled.")
        return CancelResponse.ACCEPT

    def move_command_callback(self, msg):
        """Update movement state based on /move_command topic."""
        self.current_command = msg.data

    async def execute_callback(self, goal_handle):
        """Execute the action, updating movement until the command changes."""
        movement_type = goal_handle.request.movement
        self.get_logger().info(f"Executing movement: {movement_type}")

        feedback_msg = Move.Feedback()
        rate = self.create_rate(1)  # 1 Hz update rate

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger.info("Goal was canceled.")
                goal_handle.canceled()
                return Move.result(success=False)
            
            if self.current_command != movement_type:
                self.get_logger().info(f"Command changed ({self.current_command}), stopping action.")
                goal_handle.succeed()
                return Move.Result(success=True)

            # Publish feedback
            feedback_msg.status = f"Executing {movement_type}"
            goal_handle.publish_feedback(feedback_msg)

            # Publish status message
            feedback_status = String()
            feedback_status.data = f"Executing {movement_type}"
            self.feedback_publisher.publish(feedback_status)

            self.get_logger().info(f"Executing {movement_type}...")
            rate.sleep()
        
        # Populate result message
        result = Move.Result()
        result.success = feedback_msg.success

        self.get_logger().info('Returning result: {0}'.format(result.success))

        return result



def main(args=None):
    rclpy.init(args=args)

    node = MoveActionServer()

    rclpy.spin(node)


if __name__ == '__main__':
    main()