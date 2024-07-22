Header header
float32 goal_x # mm
float32 goal_y # mm
float32 goal_z # mm
float32 goal_r # mm
float32 moving_speed # mm/s or deg/s
float32 target_speed_ratio # 0-1, where 0 represents zero speed passing the target and 1 represents max speed passing
int8 hand_type # left handed or right handed

float32 claw_postion
float32 claw_velocity
float32 claw_current

int8 LEFT_HANDED = -1
int8 RIGHT_HANDED = 1

#  return values
int16 INVALID = 0 # executing the other commands
int16 VALID = 1 # succeed and moving
int16 SPEED_VALID = 2 # set speed is negative
int16 UNINITED = 3
int16 UNARRIVABLE = 4
int16 SERVOOFF = 6
int16 WRONGHANDTYPE = 7
int16 EMERGENCYSTOP = 99
int16 WRONGPARAMETER = 101
int16 COLLISION = 102
int16 AXISRESET = 103
