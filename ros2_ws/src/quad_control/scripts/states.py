"""Predefined states for the quadruped robot and its interaction with the environment."""

# Enum-like representation for quadruped robot states
TURNING = 0
"""
The robot is actively turning to adjust its direction.
"""

HOME1 = 1
"""
The robot is in its initial home position.
"""

STOPPED_TURNING = 2
"""
The robot has completed its turn and is stationary.
"""

WALK_TO_ROLL = 3
"""
The robot is transitioning from walking mode to rolling mode.
"""

AT_ROLL_STATIONARY = 4
"""
The robot is at the rolling position but not in motion.
"""

ROLLING = 5
"""
The robot is actively rolling forward.
"""

KNOCKED_OVER_PINS = 6
"""
The robot has completed rolling and knocked over an object (e.g., pins in a bowling scenario).
"""

STOPPED_ROLLING = 7
"""
The robot has finished rolling and come to a stop.
"""