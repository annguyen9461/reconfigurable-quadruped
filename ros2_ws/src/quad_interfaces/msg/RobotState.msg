# Current robot movement state
string current_state  # "turning", "stopped", "home1", etc.

# Whether the robot is moving
bool is_moving

# Current motor positions
int32[12] motor_positions