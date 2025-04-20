# Transformerbot Control System

This README describes the usage of the control system for the quadruped robot, including both walking and rolling modes. The ROS2 code also performs a autonomous bowling using computer vision (YOLO) and machine learning.

## Demo
https://github.com/user-attachments/assets/16f294a2-c4f4-4af3-99e2-1b8f990bea4d

## Overview

The robot has two main modes of operation:
1. **Walking Mode** - Standard quadruped locomotion on four legs
2. **Rolling Mode** - Transformation into a circular shape for efficient rolling locomotion

The robot has 12 servo motors, with 3 motors per leg. Each leg has the following motors:
- **Roll Motor** - Controls up/down movement
- **Yaw Motor** - Controls side-to-side movement
- **Fold Motor** - Controls folding/unfolding of the leg

## Quickstart Option 1: Pure C++ Control

### Overall Control
```bash
cd self-reconfigurable-quadruped/c++/control/sync_read_write
make
./sync_read_write
# then type `h1` command to move to home position for walking
```

### Rolling Mode
```bash
# Terminal 1
cd self-reconfigurable-quadruped/c++/control/sync_read_write
make
./sync_read_write
# type `h1` command to move to home position for walking
# then type `hcir` command to transform to rolling mode
```
```bash
# Terminal 2
cd self-reconfigurable-quadruped/c++/control/rolling_imu
make
./roll
```

## Basic Usage

The system provides a command-line interface that accepts various commands.

## Command Reference

### Common Commands

| Command | Description |
|---------|-------------|
| `get` | Scan and display all connected motors with their current positions |
| `exit` | Exit the program |
| `h1` | Move to home position for walking|
| `ali` | Move to aligned position |
| `en X Y Z` | Enable torque for motors with IDs X, Y, Z |
| `d X Y Z` | Disable torque for motors with IDs X, Y, Z |
| `set ID:pos ID:pos ...` | Directly set motor positions (e.g., `set 1:2048 2:3000`) |

### Individual Leg Control Commands

| Command | Description |
|---------|-------------|
| `up X:Y Z ...` | Move legs Y, Z up by X degrees (e.g., `up 30:1 3`) |
| `down X:Y Z ...` | Move legs Y, Z down by X degrees |
| `cw X:Y Z ...` | Rotate legs Y, Z clockwise by X degrees |
| `ccw X:Y Z ...` | Rotate legs Y, Z counter-clockwise by X degrees |
| `fcw X:Y Z ...` | Fold legs Y, Z clockwise by X degrees |
| `fccw X:Y Z ...` | Fold legs Y, Z counter-clockwise by X degrees |

### Walking Mode Commands

| Command | Description |
|---------|-------------|
| `crawl` | Begin crawling gait (continuous) |
| `ri` | Turn right (continuous) |
| `le` | Turn left (continuous) |
| `hcir` | Transform from walking to rolling shape |

### Rolling Mode Commands

| Command | Description |
|---------|-------------|
| `cir` | Move to perfect circle position |
| `cirh` | Transform from rolling back to walking configuration |

## Leg Numbering

The robot uses the following leg numbering convention:
- Leg 1: Front-left
- Leg 2: Front-right
- Leg 3: Rear-left
- Leg 4: Rear-right

When using the `up`, `down`, `cw`, `ccw`, `fcw`, and `fccw` commands, you reference these leg numbers.

## Rolling Transformation Sequence

For transformation from walking to rolling (`hcir`):
1. The robot first moves to aligned position
2. Then transitions to circular form through intermediate positions

For transformation from rolling to walking (`cirh`):
1. Moves from circle to transition position
2. Then unfolds legs sequentially
3. Finally settles into walking stance

## Auto-Rolling Mode

When using the rolling executable, the robot has an automatic mode where it detects orientation using the IMU sensor and automatically propels the appropriate side. This allows the robot to continue rolling even when its orientation changes.

## Troubleshooting

If motors are not responding:
1. Check connections and power 
3. If a motor shows errors, try disabling and re-enabling its torque, switching the U2D2 off and on, and/or unplugging the power source to the U2D2
4. Run ./sync_read_write again and run `h1` command to reset the robot to its home position

If the robot falls during rolling:
1. Cancel ./roll
2. Return to circle position with `cir`
3. Run ./roll again

## Quickstart Option 2: Autonomous Bowling

### ROS2-Based Autonomous Bowling System

This setup enables autonomous bowling using computer vision (YOLO) and machine learning. The robot uses IMU data and visual feedback to find, roll toward, and knock down bowling pins.

ROS2 version: Jazzy

```bash
# DYNAMIXEL requirements
sudo apt install ros-jazzy-dynamixel-sdk ros-jazzy-dynamixel-sdk-custom-interfaces
```

#### Step 1: Start the `quad_motor_control` Node on the RPi5 through ssh from the laptop
- The laptop and the pi must have the same wifi.

quad_motor_control node:
- Controls the motors and reads IMU data for rolling mode
- Publishes motor positions and subscribes to rolling commands

```bash
# On laptop - RPi5 Terminal
ssh <rpi_name>@<ip_address>
cd self-reconfigurable-quadruped/ros2_ws
colcon build
source install/setup.bash
ros2 run quad_motor_control quad_motor_control
```

#### Step 2: Set up a virtual environment for the computer vision task

```bash
# On laptop
python3 -m venv venv       # create the venv
source venv/bin/activate   # activate it
pip install -r requirements.txt  # install everything from the file
```

#### Step 3: Start Action Server, Client, and YOLO Node on the laptop

These nodes:
- Handle motion planning and execution
- Use YOLO to detect and count bowling pins

```bash
# On laptop - Terminal 1
cd self-reconfigurable-quadruped/ros2_ws
colcon build
source install/setup.bash
ros2 run quad_control move_action_server &
ros2 run quad_control move_action_client &
```
```bash
# On laptop - Terminal 2
cd self-reconfigurable-quadruped/ros2_ws
source install/setup.bash
# Activate a virtual environment to access YOLO from Ultralytics
source venv/bin/activate
ros2 run quad_vision yolo_node
```
#### System Behavior

Once all nodes are active, the system will:

- Command the robot to turn in place while scanning for pins
- Use YOLO to detect if **2 or more pins** are visible for **3 or more seconds**
- Transform into the rolling configuration
- Roll forward to knock down the pins
