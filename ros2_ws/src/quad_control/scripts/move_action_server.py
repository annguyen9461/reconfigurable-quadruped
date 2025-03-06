#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from quad_interfaces.action import Move
from quad_interfaces.msg import RobotState
from quad_interfaces.msg import MotorPositions
from std_msgs.msg import String

import asyncio
import time
from rclpy.executors import MultiThreadedExecutor

class MoveActionServer(Node):
    # Enum-like representation for robot states
    TURNING = 0
    STOPPED_TURNING = 1
    WALK_TO_ROLL = 2
    AT_ROLL_STATIONARY = 3
    ROLLING = 4
    KNOCKED_OVER_PINS = 5
    STOPPED_ROLLING = 6

    def __init__(self):
        super().__init__('move_action_server')
        self._action_server = ActionServer(
            self,
            Move,
            'move',
            execute_callback=self.execute_callback,
        )
        
        self.current_command = "stop" # Default movement state
        self.current_motor_pos = [None]*12

        # Subscribers
        self.motor_positions_subscriber = self.create_subscription(
            MotorPositions,
            '/motor_positions',
            self.get_motor_pos,
            10
        )

        # Publishers
        self.state_publisher = self.create_publisher(RobotState, '/robot_state', 10)

        # Define home position to stop and stay when turning
        self.home_tiptoe = [
            2745, 2187, 3062, 1343, 1890, 1025, 2752, 2190, 3072, 2429, 1864, 1050
        ]
        self.perfect_cir = [
            2040, 1098, 3081, 2054, 2997, 1007, 2041, 2993, 1045, 3054, 1095, 3091
        ]

        self.position_threshold = 10  # Allowed error margin

        self.get_logger().info("MoveActionServer is ready.")

    def get_motor_pos(self, msg):
        """Updates the current motor positions from the MotorPositions message."""
        self.current_motor_pos = [
            msg.motor1_position, msg.motor2_position, msg.motor3_position,
            msg.motor4_position, msg.motor5_position, msg.motor6_position,
            msg.motor7_position, msg.motor8_position, msg.motor9_position,
            msg.motor10_position, msg.motor11_position, msg.motor12_position
        ]

    def is_at_home_position(self):
        """Checks if the robot's motors are within the threshold of the home position."""
        for i in range(12):
            if self.current_motor_pos[i] is None:  # Ensure motor positions are initialized
                self.get_logger().warn(f"Motor {i+1} position is None! Skipping check.")
                return False
        
            difference = abs(self.current_motor_pos[i] - self.home_tiptoe[i])
            if difference > self.position_threshold:
                self.get_logger().info(f"Motor {i+1} is off by {difference} ticks")
                return False
        return True

    def is_at_roll_position(self):
        """Checks if the robot's motors are within the threshold of the home position."""
        for i in range(12):
            if self.current_motor_pos[i] is None:  # Ensure motor positions are initialized
                self.get_logger().warn(f"Motor {i+1} position is None! Skipping check.")
                return False
            
            difference = abs(self.current_motor_pos[i] - self.perfect_cir[i])
            if difference > self.position_threshold:
                self.get_logger().info(f"Motor {i+1} is off by {difference} ticks")
                return False
        return True

    def publish_robot_state(self, state):
        """Publishes the robot's current state to the `/robot_state` topic."""
        msg = RobotState()
        msg.current_state = state
        self.state_publisher.publish(msg)

    async def execute_callback(self, goal_handle):
        """Executes the action, checking movement status and publishing state."""
        movement_type = goal_handle.request.movement
        self.get_logger().info(f"Executing movement: {movement_type}")
        
         # Append the seeds for the Fibonacci sequence
        # feedback_msg = Move.Feedback()
        result = Move.Result()

        # Initial feedback message
        # feedback_msg.status_message = f"Starting {movement_type}..."
        # feedback_msg.still_moving = True
        # goal_handle.publish_feedback(feedback_msg)

        if movement_type == "turn":
            self.publish_robot_state(self.TURNING)
            self.get_logger().info("Starting turn movement...")
            
            # Wait for robot to reach home position
            while not self.is_at_home_position():
                # feedback_msg.status_message = "Turning..."
                # feedback_msg.still_moving = True
                # goal_handle.publish_feedback(feedback_msg)
                # await asyncio.sleep(0.5)  # wait and check again
                time.sleep(0.5)
            
            self.get_logger().info("Turn completed. At home position.")
            self.publish_robot_state(self.STOPPED_TURNING)

        elif movement_type == "stop_turning":
            self.publish_robot_state(self.STOPPED_TURNING)
            self.get_logger().info("Server log: Stopped turning.")
        
        elif movement_type == "hcir":
            self.publish_robot_state(self.WALK_TO_ROLL)
            self.get_logger().info("Transitioning to rolling mode...")
            
            # Wait for robot to reach rolling position
            while not self.is_at_roll_position():
                # feedback_msg.status_message = "Trans to roll..."
                # feedback_msg.still_moving = True
                # goal_handle.publish_feedback(feedback_msg)
                # await asyncio.sleep(0.5)  # wait and check again
                time.sleep(0.5)
            
            self.get_logger().info("Transition to roll config completed.")
            self.publish_robot_state(self.AT_ROLL_STATIONARY)
        
        elif movement_type == "rolling":
            self.publish_robot_state(self.ROLLING)
            self.get_logger().info("Rolling...")
            # await asyncio.sleep(3)  # Simulate rolling time
            time.sleep(0.5)
            self.publish_robot_state(self.KNOCKED_OVER_PINS)

        elif movement_type == "stop_rolling":
            self.publish_robot_state(self.STOPPED_ROLLING)
            self.get_logger().info("Stopped rolling.")

        # Final feedback and result
        # feedback_msg.status_message = f"{movement_type} completed"
        # feedback_msg.still_moving = False
        # goal_handle.publish_feedback(feedback_msg)

        # Mark goal as succeeded
        goal_handle.succeed()
        result.arrived = True
        return result


def main(args=None):
    rclpy.init(args=args)

    node = MoveActionServer()

    # rclpy.spin(node) 
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)


if __name__ == '__main__':
    main()