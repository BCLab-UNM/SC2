int32 PATH_IS_CLEAR    = 0
int32 LIDAR_LEFT       = 1
int32 LIDAR_RIGHT      = 2
int32 LIDAR_CENTER     = 4
int32 LIDAR_BLOCK      = 8
int32 VOLATILE         = 256
int32 TAG_HOME         = 512
int32 TAG_TARGET       = 1024
int32 HOME_CORNER      = 2048

# bitmasks
int32 IS_LIDAR         = 15
int32 IS_VOLATILE      = 256
int32 IS_VISION        = 3840

# convenience masks
# All vision obstacles that are generally safe to ignore. These are TAG_TARGET,
# TAG_HOME, and HOME_CORNER, but !!NOT!! INSIDE_HOME
int32 VISION_SAFE      = 2816
# Both TAG_HOME and HOME_CORNER
int32 VISION_HOME      = 2560

int32 msg 
int32 mask
