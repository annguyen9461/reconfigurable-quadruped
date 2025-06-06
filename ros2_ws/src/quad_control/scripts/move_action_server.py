#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from quad_interfaces.action import Move
from quad_interfaces.msg import RobotState
from quad_interfaces.msg import MotorPositions
from std_msgs.msg import String

import time

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

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
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        
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
            2745, 2228, 3062, 1343, 1890, 1025, 2752, 2190, 3072, 2429, 1864, 1050
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

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        """Executes the action, checking movement status and publishing state."""
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            return Move.Result()
        
        movement_type = goal_handle.request.movement
        self.get_logger().info(f"Executing movement: {movement_type}")

        if movement_type == "turning":
            self.publish_robot_state(self.TURNING)
            self.get_logger().info("Starting turn movement...")
            
            # Wait for robot to reach home position
            while not self.is_at_target_config(self.home_tiptoe):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info("Turning goal was canceled early!")
                    return Move.Result()  # Exit immediately

                time.sleep(0.05)  # Reduce sleep time for faster response
            
            self.get_logger().info("Turn completed. At home position.")
            self.publish_robot_state(self.HOME1)

        elif movement_type == "stop_turning":
            self.publish_robot_state(self.STOPPED_TURNING)
            self.get_logger().info("Server log: Stopped turning.")
        
        elif movement_type == "hcir":
            self.publish_robot_state(self.WALK_TO_ROLL)
            self.get_logger().info("Transitioning to rolling mode...")
            
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

        # Mark goal as succeeded
        goal_handle.succeed()
        
        # Populate result message
        result = Move.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    node = MoveActionServer()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(node, executor=executor)

    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()