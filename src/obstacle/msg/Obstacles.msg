int32 PATH_IS_CLEAR    = 0
int32 LIDAR_LEFT       = 1
int32 LIDAR_RIGHT      = 2
int32 LIDAR_CENTER     = 4
int32 LIDAR_BLOCK      = 8
int32 VOLATILE         = 256
int32 TAG_HOME         = 512
int32 TAG_PRIORI       = 1024
int32 HOME_FIDUCIAL      = 2048

# Tracking HaulerMsg.msg's comment
# in srcp2-competitors/ros_workspace/install/shares/rcp2_msgs/msg/
uint8 ice           = 0
uint8 ethene        = 1
uint8 methane       = 2
uint8 carbon_mono   = 3
uint8 carbon_dio    = 4
uint8 ammonia       = 5
uint8 hydrogen_sul  = 6
uint8 sulfur_dio    = 7

# bitmasks
int32 IS_LIDAR         = 15 #  0b000000001111
int32 IS_VOLATILE      = 256 # 0b000100000000
int32 IS_VISION        = 3584 #0b111000000000

# convenience masks
# All vision obstacles that are generally safe to ignore. These are TAG_TARGET,
# TAG_HOME, and HOME_CORNER, but !!NOT!! INSIDE_HOME
int32 VISION_SAFE      = 2816
# Both TAG_HOME and HOME_CORNER
int32 VISION_HOME      = 2560

int32 msg 
int32 mask
int32 data #used for vol_type_index
float32 distance
