# Robot state enum as an integer
uint8 TURNING = 0
uint8 STOPPED = 1
uint8 WALK_TO_ROLL = 2
uint8 ROLLING = 3
uint8 ROLL_TO_WALK = 4
uint8 HOME1 = 5

# Current state of the robot
uint8 current_state  

# Whether the robot is moving
bool is_moving

# Current motor positions
int32[12] motor_positions
