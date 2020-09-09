# Integer ID number for the object being detected
#     ID's are defined in Obstacles.msg in the obstacle package:
#         - VOLATILE
#         - TAG_HOME
#         - TAG_PRIORI
#         - HOME_FIDUCIAL
#         - CUBESAT
int32 detection_id




# Heading needed to turn to face the detected object in radians
float32 heading




# Estimated distance to the detected object in metres
float32 distance



# Estimated XYZ coordinates of the detected object
float32 x
float32 y
float32 z
