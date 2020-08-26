int32 PATH_IS_CLEAR    = 0
int32 LIDAR_LEFT       = 1
int32 LIDAR_RIGHT      = 2
int32 LIDAR_CENTER     = 4
int32 LIDAR_BLOCK      = 8
int32 VISION_LEFT      = 8
int32 VISION_RIGHT     = 16
int32 VISION_CENTER    = 32
int32 VISION_BLOCK     = 64
int32 VISION_VOLATILE  = 128
int32 VOLATILE         = 256
int32 TAG_HOME         = 512
int32 TAG_PRIORI       = 1024
int32 HOME_FIDUCIAL    = 2048

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
int32 IS_VISION_O      = 240 # 0b000011110000
int32 IS_VOLATILE      = 256 # 0b000100000000
int32 IS_VISION_T      = 3584 #0b111000000000

# convenience masks
# All IS_VISION_T and IS_VOLATILE obstacles that are generally safe to ignore
int32 SAFE      = 3840 # 0b111100000000

int32 msg 
int32 mask
int32 data #used for vol_type_index
float32 distance
