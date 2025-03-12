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

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class MoveActionClient(Node):
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
        super().__init__('move_action_client')
        # QoS settings for "latched" topic (last message is saved)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # Ensures delivery
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # Retains last message
            depth=1  # Only keep the latest message
        )

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
        # self.config_publisher = self.create_publisher(SetConfig, '/set_config', 10)
        self.config_publisher = self.create_publisher(SetConfig, '/set_config', qos_profile)

        # Timer for continuous turning
        self.turning_timer = None

        # State tracking
        self.pin_detected_time = None  # Track when pins >=2 are first detected
        self.pin_threshold = 5.0  # Seconds that pins must be detected
        self.found_enough_pins = False
        self.curr_state = self.TURNING
        self.last_published_config = None   # Track last published config ID
        self.action_in_progress = False     # Flag to prevent multiple action calls

        # Start in turning state
        self.get_logger().info("Starting in turning state")
        self.create_turning_timer()

    def create_turning_timer(self):
        """Create a timer that will repeatedly send turning commands"""
        if self.turning_timer is None:
            self.turning_timer = self.create_timer(1.0, self.turning_callback)
            self.get_logger().info("Created turning timer")

    def destroy_turning_timer(self):
        """Stop the turning timer when no longer needed"""
        if self.turning_timer is not None:
            self.turning_timer.cancel()
            self.turning_timer = None
            self.get_logger().info("Destroyed turning timer")

    def turning_callback(self):
        """Callback for the turning timer - sends turn command regularly"""
        if self.curr_state == self.HOME1:
            self.action_in_progress = False
        if not self.found_enough_pins and self.curr_state < self.STOPPED_TURNING:
            # Publish turning config first, as that's what motors will use
            self.publish_config_once(5)  # Config ID 5 for turning
            
            # Send action goal if no action is currently in progress
            if not self.action_in_progress:
                self.send_goal("turning")
        else:
            self.destroy_turning_timer()

    def publish_config_once(self, config_id):
        """Publishes a configuration message **only if it's new**."""
        if self.last_published_config == config_id:
            # self.get_logger().info(f"Config {config_id} already published. Skipping duplicate.")
            return  # Skip duplicate publication
        
        config_msg = SetConfig()
        config_msg.config_id = config_id
        self.config_publisher.publish(config_msg)
        self.last_published_config = config_id  # Update last published config ID
        self.get_logger().info(f"Published new config: {config_id}")

    def robot_state_callback(self, msg):
        """Updates the robot's state based on published `/robot_state` topic."""
        previous_state = self.curr_state
        self.curr_state = msg.current_state
        
        if previous_state != self.curr_state:
            self.get_logger().info(f"State changed from {previous_state} to {self.curr_state}")
            
            # React to state changes
            if self.curr_state == self.HOME1:
                # We've reached HOME1 during turning - can continue turning
                self.action_in_progress = False
                
            elif self.curr_state == self.STOPPED_TURNING and self.found_enough_pins:
                # Robot has stopped turning, now transition to roll
                self.transition_to_roll()
                
            elif self.curr_state == self.AT_ROLL_STATIONARY:
                # We've reached roll position, now start rolling
                self.start_rolling()
                
            elif self.curr_state == self.KNOCKED_OVER_PINS:
                # We've knocked over pins, stop rolling
                self.stop_rolling()
                
            elif self.curr_state == self.STOPPED_ROLLING:
                # We're done with the sequence
                self.get_logger().info("Completed entire bowling sequence!")

    def command_callback(self, msg):
        """Decides action based on the number of bowling pins detected."""
        bowling_pin_count = msg.data

        if not self.found_enough_pins:
            self.get_logger().info(f"Received pin count: {bowling_pin_count}")
            curr_time = time.time()

            if bowling_pin_count >= 2:  
                if self.pin_detected_time == None:
                    # First detection, start timer
                    self.pin_detected_time = curr_time
                    self.get_logger().info("Pins detected! Starting 5s timer...")
                elif (curr_time - self.pin_detected_time) >= self.pin_threshold:
                    # Pins have been detected continuously for 5 seconds
                    self.stop_turning()
                    return  # No need to continue processing
            else:
                # Reset timer if pins drop below threshold before 5 seconds
                if self.pin_detected_time is not None:
                    self.get_logger().info("Pins dropped below threshold, resetting timer.")
                    self.pin_detected_time = None
            
    def send_goal(self, movement_type):
        """Send an action goal to the move action server."""
        # Don't send if an action is already in progress
        if self.action_in_progress and movement_type == "turning":
            return False
        
        goal_msg = Move.Goal()
        goal_msg.movement = movement_type

        self.get_logger().info(f"Sending goal: {movement_type}")

        # Wait for server (with timeout)
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Action server not available, trying again later")
            return False

        # Mark that we're sending an action
        self.action_in_progress = True

        # Send goal asynchronously
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        """Handle response from the action server"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.action_in_progress = False
            return
        
        self.get_logger().info('Goal accepted')

        # Request the result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle the result from the action server"""
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().info(f'Goal failed with status: {status}')

        # For turning, reset the action flag when we reach HOME1 state
        # For other actions, we can reset it here
        if self.curr_state != self.TURNING and self.curr_state != self.HOME1:
            self.action_in_progress = False

    def stop_turning(self):
        """Stop turning and transition to Home1 configuration."""
        self.found_enough_pins = True
        self.destroy_turning_timer()  # Stop the turning timer

        self.get_logger().info("Pin detected! Stopping turn and transitioning to home1.")
        # First publish the config to stop motion
        self.publish_config_once(1)  # Home1 configuration
        
        # Then send the action goal
        self.send_goal("stop_turning")
    
    def transition_to_roll(self):
        """Transition to rolling configuration."""
        self.get_logger().info("Transitioning to roll configuration")
        
        # First publish the config for roll
        self.publish_config_once(3)  # Roll configuration
        
        # Then send the action to transition to roll
        self.action_in_progress = False  # Allow new action
        self.send_goal("hcir")

    def start_rolling(self):
        """Start rolling"""
        self.get_logger().info("Starting rolling motion")
        self.action_in_progress = False  # Allow new action
        self.send_goal("rolling")

    def stop_rolling(self):
        """Stop rolling"""
        self.get_logger().info("Stopping rolling motion")
        self.action_in_progress = False  # Allow new action
        self.send_goal("stop_rolling")

def main(args=None):
    rclpy.init(args=args)
    client_node = MoveActionClient()
    rclpy.spin(client_node)
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    