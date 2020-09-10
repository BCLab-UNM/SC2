# Integer ID number for the object being detected
#     ID's are defined in Obstacles.msg in the obstacle package:
#         - VOLATILE
#         - TAG_HOME
#         - TAG_PRIORI
#         - HOME_FIDUCIAL
#         - CUBESAT
int32 detection_id




# Heading in radians needed to turn to face the detected object
#     - the horizontal field of view from left to right is estimated as [-60 deg, 60 deg]
#     - if this heading is negative, the robot needs to turn right to face the detected object
#     - if this heading is positive, the robot needs to turn left to face the detected object
#     - the most accurate XYZ reading will be when this heading is minimized as close to 0 as possible
float32 heading




# Estimated distance to the detected object in metres
#     - this is the straight line distance from the robot to the cubesat
#
#                     + cubesat
#                    /|
#        distance > / |
#                  /  |
#           robot +---+
#
float32 distance




# Estimated XYZ coordinates of the detected object
float32 x
float32 y
float32 z
