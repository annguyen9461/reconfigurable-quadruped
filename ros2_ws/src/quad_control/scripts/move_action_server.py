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

class MoveActionServer(Node):
    # Enum-like representation for robot states
    TURNING = 0
    HOME1 = 1
    STOPPED_TURNING = 2
    WALK_TO_ROLL = 3
    AT_ROLL_STATIONARY = 4
    ROLLING = 5
    KNOCKED_OVER_PINS = 6
    STOPPED_ROLLING = 7

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

        self.position_threshold = 20  # Allowed error margin

        self.get_logger().info("MoveActionServer is ready.")

    def get_motor_pos(self, msg):
        """Updates the current motor positions from the MotorPositions message."""
        self.current_motor_pos = [
            msg.motor1_position, msg.motor2_position, msg.motor3_position,
            msg.motor4_position, msg.motor5_position, msg.motor6_position,
            msg.motor7_position, msg.motor8_position, msg.motor9_position,
            msg.motor10_position, msg.motor11_position, msg.motor12_position
        ]

    def is_at_target_config(self, target_config):
        """Checks if the robot's motors are within the threshold of the home position."""
        sum_diff = 0
        for i in range(12):
            if self.current_motor_pos[i] is None:  # Ensure motor positions are initialized
                self.get_logger().warn(f"Motor {i+1} position is None! Skipping check.")
                return False
        
            difference = abs(self.current_motor_pos[i] - target_config[i])
            sum_diff += difference
            if difference > self.position_threshold:
                self.get_logger().info(f"Motor {i+1} is off by {difference} ticks")
        if sum_diff > self.position_threshold*12:
            self.get_logger().info(f"Motors are off by {sum_diff} ticks")
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
        
        #  # Append the seeds for the Fibonacci sequence
        # feedback_msg = Move.Feedback()
        

        # # Initial feedback message
        # feedback_msg.status_message = f"Starting {movement_type}..."
        # feedback_msg.still_moving = True
        # goal_handle.publish_feedback(feedback_msg)

        if movement_type == "turning":
            self.publish_robot_state(self.TURNING)
            self.get_logger().info("Starting turn movement...")
            
            # Wait for robot to reach home position
            while not self.is_at_target_config(self.home_tiptoe):
                # feedback_msg.status_message = "Turning..."
                # feedback_msg.still_moving = True
                # goal_handle.publish_feedback(feedback_msg)
                time.sleep(0.2)
            
            self.get_logger().info("Turn completed. At home position.")
            self.publish_robot_state(self.HOME1)

        elif movement_type == "stop_turning":
            self.publish_robot_state(self.STOPPED_TURNING)
            self.get_logger().info("Server log: Stopped turning.")
        
        elif movement_type == "hcir":
            self.publish_robot_state(self.WALK_TO_ROLL)
            self.get_logger().info("Transitioning to rolling mode...")
            
            # Wait for robot to reach rolling position
            while not self.is_at_target_config(self.perfect_cir):
                # feedback_msg.status_message = "Trans to roll..."
                # feedback_msg.still_moving = True
                # goal_handle.publish_feedback(feedback_msg)
                time.sleep(0.5)
            
            self.get_logger().info("Transition to roll config completed.")
            self.publish_robot_state(self.AT_ROLL_STATIONARY)
        
        elif movement_type == "rolling":
            self.publish_robot_state(self.ROLLING)
            self.get_logger().info("Rolling...")
            time.sleep(0.5)
            self.publish_robot_state(self.KNOCKED_OVER_PINS)

        elif movement_type == "stop_rolling":
            self.publish_robot_state(self.STOPPED_ROLLING)
            self.get_logger().info("Stopped rolling.")

        # # Final feedback and result
        # feedback_msg.status_message = f"{movement_type} completed"
        # feedback_msg.still_moving = False
        # goal_handle.publish_feedback(feedback_msg)

        # Mark goal as succeeded
        goal_handle.succeed()
        
        # Populate result message
        result = Move.Result()
        # self.get_logger().info(f"Result object attributes: {dir(result)}")
        # self.get_logger().info(f"Result object type: {type(result)}")
        # result.arrived = True
        # self.get_logger().info('Returning result: {0}'.format(result.arrived))
        return result


def main(args=None):
    rclpy.init(args=args)

    node = MoveActionServer()

    # # Test publishing a message every 2 seconds
    # while rclpy.ok():
    #     node.publish_robot_state(node.TURNING)
    #     node.get_logger().info("Published test state: TURNING")
    #     time.sleep(2)

    rclpy.spin(node) 


if __name__ == '__main__':
    main()