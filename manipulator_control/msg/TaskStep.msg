uint8 MOVE_POSE=0
uint8 GRIPPER_COMMAND=1
uint8 WAIT=2

uint8 type

# MOVE_POSE
geometry_msgs/Pose pose
string frame_id
bool is_cartesian

# GRIPPER_COMMAND
bool close_gripper
int32 speed
int32 force

# WAIT
float32 wait_sec