int32 PATH_IS_CLEAR    = 0
int32 LIDAR_LEFT       = 1
int32 LIDAR_RIGHT      = 2
int32 LIDAR_CENTER     = 4
int32 LIDAR_BLOCK      = 8
int32 VISION_LEFT      = 8
int32 VISION_RIGHT     = 16
int32 VISION_CENTER    = 32
int32 VISION_BLOCK     = 64
int32 VOLATILE         = 128
int32 VISION_VOLATILE  = 256
int32 CUBESAT          = 512
int32 HOME_LEG         = 1024
int32 HOME_FIDUCIAL    = 2048

# Note the rules still say ethene but the simulation "lib*.so"s create ethane
uint8 ice               = 0
uint8 ethane            = 1 
uint8 methane           = 2
uint8 methanol          = 3
uint8 carbon_dioxide    = 4
uint8 ammonia           = 5
uint8 hydrogen_sulfite  = 6
uint8 sulfur_dioxide    = 7
uint8 regolith          = 8

# bitmasks
int32 IS_LIDAR         = 15 #  0b000000001111 LIDAR_*
int32 IS_VISION_O      = 240 # 0b000011110000 VISION_*
int32 IS_VOLATILE      = 128 # 0b000010000000 VOLATILE
int32 IS_VISION_T      = 3840 #0b111100000000 VISION_VOLATILE CUBESAT HOME_LEG HOME_FIDUCIAL

# convenience masks
# All IS_VISION_T and IS_VOLATILE obstacles that are generally safe to ignore
int32 SAFE      = 3968 # 0b111110000000

int32 msg 
int32 mask
int32 data #used for vol_type_index
float32 distance
