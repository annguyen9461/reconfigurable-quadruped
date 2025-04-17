# Transformerbot Control System

This README describes the usage of the control system for the quadruped robot, including both walking and rolling modes. The ROS2 code also performs a autonomous bowling using computer vision (YOLO) and machine learning.

## Overview

The robot has two main modes of operation:
1. **Walking Mode** - Standard quadruped locomotion on four legs
2. **Rolling Mode** - Transformation into a circular shape for efficient rolling locomotion

The robot has 12 servo motors, with 3 motors per leg. Each leg has the following motors:
- **Roll Motor** - Controls up/down movement
- **Yaw Motor** - Controls side-to-side movement
- **Fold Motor** - Controls folding/unfolding of the leg

## Setup & Compilation (Option1: Pure C++ Control)

### Overall Control
- cd into sync_read_write folder
- make
- run ./sync_read_write executable

### Rolling Mode
- run ./sync_read_write executable and transform the robot using hcr (home to circle)
- cd into rolling_imu folder
- make
- run ./roll executable

## Basic Usage

The system provides a command-line interface that accepts various commands.

## Command Reference

### Common Commands

| Command | Description |
|---------|-------------|
| `get` | Scan and display all connected motors with their current positions |
| `exit` | Exit the program |
| `h1` | Move to home tiptoe position |
| `h2` | Move to thin tiptoe position |
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
| `rfw` | Roll forward continuously (alternating yellow/blue propulsion) |
| `rfy` | Execute single yellow side propulsion |
| `rfb` | Execute single blue side propulsion |
| `rpy` | Execute stronger yellow side propulsion |
| `rpb` | Execute stronger blue side propulsion |
| `cirh` | Transform from rolling back to walking configuration |
| `recol` | Recover from left side on ground |

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

## Examples

```
# Move to home position
h1

# Move legs 1 and 3 up by 20 degrees
up 20:1 3

# Set motor 5 to position 2500
set 5:2500

# Start continuous rolling
rfw

# Transform from walking to rolling
hcir

# Transform from rolling to walking
cirh
```

## Troubleshooting

If motors are not responding:
1. Check connections and power
2. Verify motors are enabled with `get` command 
3. If needed, enable torque with `en` command for specific motors
4. If a motor shows errors, try disabling and re-enabling its torque

If the robot falls during rolling:
1. Return to circle position with `cir`
2. Resume rolling with `rfw` or propel once with `rfy`/`rfb`

## Setup & Compilation (Option 2: Autonomous Bowling using ROS2)

### ROS2-Based Autonomous Bowling System

This system enables autonomous bowling using computer vision (YOLO) and machine learning. The robot uses IMU data and visual feedback to find, roll toward, and knock down bowling pins.

#### Step 1: Start the `quad_motor_control` Node on the RPi5

This node:
- Controls the motors and reads IMU data for rolling mode
- Publishes motor positions and subscribes to rolling commands

```bash
# On RPi5
source ~/ros2_ws/install/setup.bash
ros2 run quad_motor_control quad_motor_control
```

#### Step 2: Start Action Server, Client, and YOLO Node on Laptop

These nodes:
- Handle motion planning and execution
- Use YOLO to detect and count bowling pins

```bash
# On laptop
source ~/ros2_ws/install/setup.bash
ros2 run quad_action move_action_server &
ros2 run quad_action move_action_client &
ros2 run quad_vision yolo_node
```

#### System Behavior

Once all nodes are active, the system will:

- Command the robot to turn in place while scanning for pins
- Use YOLO to detect if **2 or more pins** are visible for **3 or more seconds**
- Transform into the rolling configuration
- Roll forward to knock down the pins
- Stop automatically once the pins are hit
