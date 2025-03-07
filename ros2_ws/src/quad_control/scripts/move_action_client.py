#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Int32  # The topic message type
from quad_interfaces.action import Move   # The action type
from quad_interfaces.msg import SetConfig
from quad_interfaces.msg import RobotState

from action_msgs.msg import GoalStatus
import time

class MoveActionClient(Node):
    # Enum-like representation for robot states
    TURNING = 0
    STOPPED_TURNING = 1
    WALK_TO_ROLL = 2
    AT_ROLL_STATIONARY = 3
    ROLLING = 4
    KNOCKED_OVER_PINS = 5
    STOPPED_ROLLING = 6
    def __init__(self):
        super().__init__('move_action_client')
        # Create an action client
        self._action_client = ActionClient(self, Move, 'move')

        # Subscribers
        self.num_pins_subscriber = self.create_subscription(
            Int32,
            '/num_bowling_pins',
            self.command_callback,
            10
        )
        self.robot_state_subscriber = self.create_subscription(
            RobotState,
            '/robot_state',
            self.robot_state_callback,
            10
        )

        # Publishers
        self.config_publisher = self.create_publisher(SetConfig, '/set_config', 10)
        
        self.pin_detected_time = None  # Track when pins >=2 are first detected
        self.pin_threshold = 3.0  # Seconds that pins must be detected

        self.found_enough_pins = False
        self.curr_state = self.TURNING

    def robot_state_callback(self, msg):
        """Updates the robot's state based on published `/robot_state` topic."""
        self.curr_state = msg.current_state

    def command_callback(self, msg):
        """Decides action based on the number of bowling pins detected."""
        bowling_pin_count = msg.data
        self.get_logger().info(f"Received pin count: {bowling_pin_count}")

        if bowling_pin_count >= 2:  # Found a pin, stop turning
            self.stop_turning()
            return
        else:
            self.keep_turning()

    # def send_goal(self, movement_type):
    #     """Send an action goal to the move action server."""
    #     goal_msg = Move.Goal()
    #     goal_msg.movement = movement_type

    #     self.get_logger().info(f"Sending goal: {movement_type}")

    #     self._action_client.wait_for_server()

    #     return self._action_client.send_goal_async(goal_msg)

    async def send_goal(self, movement_type):
        """Send an action goal to the move action server."""
        goal_msg = Move.Goal()
        goal_msg.movement = movement_type

        self.get_logger().info(f"Waiting for action server before sending goal: {movement_type}")
    

        self.get_logger().info(f"Sending goal: {movement_type}")

        self._action_client.wait_for_server()

        future = self._action_client.send_goal_async(goal_msg)  # Request goal asynchronously
        goal_handle = await future  # Wait for goal acceptance

        if not goal_handle.accepted:
            self.get_logger().error(f"Goal {movement_type} was rejected!")
            return

        self.get_logger().info(f"Goal {movement_type} accepted. Waiting for result...")

        # Wait for the action to complete
        result_future = goal_handle.get_result_async()
        result = await result_future

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Goal {movement_type} succeeded!")
        else:
            self.get_logger().error(f"Goal {movement_type} failed with status {result.status}")
   
    def stop_turning(self):
        """Stop turning and transition to Home1 configuration."""
        self.get_logger().info("Pin detected! Stopping turn and transitioning to home1.")
        self.send_goal("stop_turning")

        # Publish to `/set_config` to stop moving
        config_msg = SetConfig()
        config_msg.config_id = 1
        self.get_logger().info("Publishing transition to home1.")
        self.config_publisher.publish(config_msg)
    
    def keep_turning(self):
        """Keep turning and until detect enough bowling pins."""
        self.get_logger().info("Not enough pins yet! Keep turning.")
        self.send_goal("turn")

        # Publish to `/set_config` to stop moving
        config_msg = SetConfig()
        config_msg.config_id = 5
        self.get_logger().info("Publishing congfig 5 to turn.")
        self.config_publisher.publish(config_msg)

    
    # async def stop_rolling(self):
    #     """Stop rolling"""
    #     await self.send_goal("stop_rolling")
    #     self.get_logger().info("Stopping rolling...")

    # async def start_rolling(self):
    #     """Start rolling"""
    #     await self.send_goal("rolling")
    #     self.get_logger().info("Starting rolling...")

    # async def transition_to_roll(self):
    #     """Transition to rolling configuration."""
    #     await self.send_goal("hcir")

    #     # Publish to `/set_config` to stop moving
    #     config_msg = SetConfig()
    #     config_msg.config_id = 3
    #     self.get_logger().info("Transitioning to roll...")
    #     self.config_publisher.publish(config_msg)

    # async def stop_turning(self):
    #     """Stop turning and transition to Home1 configuration."""
    #     self.get_logger().info("Pin detected! Stopping turn and transitioning to home1.")
    #     await self.send_goal("stop_turning")

    #     # Publish to `/set_config` to stop moving
    #     config_msg = SetConfig()
    #     config_msg.config_id = 1
    #     self.get_logger().info("Publishing transition to home1.")
    #     self.config_publisher.publish(config_msg)
    
    # async def keep_turning(self):
    #     """Keep turning and until detect enough bowling pins."""
    #     self.get_logger().info("Not enough pins yet! Keep turning.")
    #     await self.send_goal("turn")

    #     # Publish to `/set_config` to stop moving
    #     config_msg = SetConfig()
    #     config_msg.config_id = 5
    #     self.get_logger().info("Publishing congfig 5 to turn.")
    #     self.config_publisher.publish(config_msg)


def main(args=None):
    rclpy.init(args=args)
    client_node = MoveActionClient()
    rclpy.spin(client_node)
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    

    # async def handle_command(self, msg):
    #     """Processes the pin detection logic in order."""
    #     bowling_pin_count = msg.data
    #     self.get_logger().info(f"Received pin count: {bowling_pin_count}")

    #     if not self.found_enough_pins:
    #         curr_time = time.time()

    #         if bowling_pin_count >= 2:  
    #             if self.pin_detected_time == None:
    #                 # First detection, start timer
    #                 self.pin_detected_time = curr_time
    #                 self.get_logger().info("Pins detected! Starting 3s timer...")
    #             elif (curr_time - self.pin_detected_time) >= self.pin_threshold:
    #                 # Pins have been detected continuously for 3 seconds
    #                 await self.stop_turning()
    #                 self.found_enough_pins = True
    #                 return  # No need to continue processing
    #         else:
    #             # Reset timer if pins drop below threshold before 3 seconds
    #             if self.pin_detected_time is not None:
    #                 self.get_logger().info("Pins dropped below threshold, resetting timer.")
    #                 self.pin_detected_time = None
            
    #         await self.keep_turning()
    #     else:
    #         if self.curr_state == self.STOPPED_TURNING:
    #             await self.transition_to_roll()
    #         elif self.curr_state == self.AT_ROLL_STATIONARY:
    #             await self.start_rolling()
    #         elif self.curr_state == self.KNOCKED_OVER_PINS:
    #             await self.stop_rolling()

    # async def send_goal(self, movement_type):
    #     """Send an action goal to the move action server."""
    #     goal_msg = Move.Goal()
    #     goal_msg.movement = movement_type

    #     self.get_logger().info(f"Waiting for action server before sending goal: {movement_type}")
    
    #     while not self._action_client.wait_for_server(timeout_sec=1.0):
    #         self.get_logger().warn("Waiting for move action server...")


    #     self.get_logger().info(f"Sending goal: {movement_type}")

    #     future = self._action_client.send_goal_async(goal_msg)  # Request goal asynchronously
    #     goal_handle = await future  # Wait for goal acceptance

    #     if not goal_handle.accepted:
    #         self.get_logger().error(f"Goal {movement_type} was rejected!")
    #         return

    #     self.get_logger().info(f"Goal {movement_type} accepted. Waiting for result...")

    #     # Wait for the action to complete
    #     result_future = goal_handle.get_result_async()
    #     result = await result_future

    #     if result.status == GoalStatus.STATUS_SUCCEEDED:
    #         self.get_logger().info(f"Goal {movement_type} succeeded!")
    #     else:
    #         self.get_logger().error(f"Goal {movement_type} failed with status {result.status}")

   