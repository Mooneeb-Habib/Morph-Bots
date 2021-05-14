

float64[] wheel_vel    # wheel velocities
float64[] last_check   # timer check for shape formation
float64[] turn_time    # turn timer
float64[] wait_time    # waiting timer
float64[] move_time    # linear movement timer       
int32[] turn_choice    # chooses which direction to turn
bool[] turn_start      # checks whether turning has started
bool[] moving          # checks whether robot is moving
bool[] linear_move     # checks whether robot is moving forward
bool[] check           # checks whether last_check timer has been initialized
bool[] wait_flag       # checks whether robot should move or stay at current waypoint
bool[] reach           # checks if robot has reached its goal
