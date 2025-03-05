#!/usr/bin/env python3

import rclpy
import rclpy.task
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from quad_interfaces.action import Move
from quad_interfaces.srv import GetAllPositions
from quad_interfaces.msg import RobotState
from std_msgs.msg import String

class MoveActionServer(Node):
    # Enum-like representation for robot states
    TURNING = 0
    STOPPED = 1
    WALK_TO_ROLL = 2
    ROLLING = 3
    ROLL_TO_WALK = 4
    HOME1 = 5

    def __init__(self):
        super().__init__('move_action_server')
        self._action_server = ActionServer(
            self,
            Move,
            'move',
            execute_callback=self.execute_callback,
        )
        
        self.current_command = "stop" # Default movement state

        self.state_publisher = self.create_publisher(RobotState, '/robot_state', 10)

        # Client for the /get_all_positions service
        self.get_positions_client = self.create_client(GetAllPositions, 'get_all_positions')

        # Define home position to stop and stay when turning
        self.home_tiptoe = [
            2745, 2187, 3062, 1343, 1890, 1025, 2752, 2190, 3072, 2429, 1864, 1050
        ]
        self.position_threshold = 10  # Allowed error margin

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("MoveActionServer is ready.")

    async def get_motor_positions(self):
        """Calls the /get_all_positions service and returns the motor positions."""
        if not self.get_positions_client.service_is_ready():
            self.get_logger().warn("Waiting for /get_all_positions service...")
            await self.get_position_client.wait_for_service()
        
        request = GetAllPositions.Request()
        future = self.get_positions_client.call_async(request)
        await future

        if future.result() is not None:
            response = future.result
            return [
                response.motor1_position, response.motor2_position, response.motor3_position,
                response.motor4_position, response.motor5_position, response.motor6_position,
                response.motor7_position, response.motor8_position, response.motor9_position,
                response.motor10_position, response.motor11_position, response.motor12_position
            ]
        else:
            self.get_logger().error("Failed to get motor positions!")


    async def publish_robot_state(self, state):
        """Publishes the robot's current state to the `/robot_state` topic."""
        motor_positions = await self.get_motor_positions()

        msg = RobotState()
        msg.current_state = state
        msg.is_moving = motor_positions != self.home_tiptoe
        msg.motor_positions = motor_positions
        self.state_publisher.publish(msg)
        self.get_logger().info(f'Published motor positions: {msg.motor_positions}')

    async def execute_callback(self, goal_handle):
        """Executes the action, checking movement status and publishing state."""
        movement_type = goal_handle.request.movement
        self.get_logger().info(f"Executing movement: {movement_type}")
        
        # if movement_type == "turn":
        #     await self.publish_robot_state(self.TURNING, is_moving = True)
        #     self.get_logger().info("Starting turn movement...")

        #     self.get_logger().info("Turn completed. Waiting for home position...")
        
        # elif movement_type == "stop":
        #     await self.publish_robot_state(self.STOPPED, is_moving=False)

        # Populate result message
        result = Move.Result()
        return result

    def timer_callback(self):
        """Schedules an asynchronous motor position check and state update."""
        rclpy.task.using_future(self.publish_robot_state(self.TURNING))

def main(args=None):
    rclpy.init(args=args)

    node = MoveActionServer()

    rclpy.spin(node)


if __name__ == '__main__':
    main()