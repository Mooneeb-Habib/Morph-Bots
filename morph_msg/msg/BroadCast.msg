# message type to describe all information needed for current status of two wheel robots
# the dimensions of following array should be the same, each elements indicate a robot

int32[] index
int64[] hop
float64[] x
float64[] y
float64[] orientation
float64[] left_wheel_vel
float64[] right_wheel_vel
geometry_msgs/Point[] wp
geometry_msgs/Point[] next_step
geometry_msgs/Point[] T
geometry_msgs/Point[] q_u

