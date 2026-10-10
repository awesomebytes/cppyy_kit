"""ROS 2 msg definitions used by the bounded CDR adapter."""
HEADER = """std_msgs/Header header
"""
DEPENDENCIES = """================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""
POSE = HEADER + """geometry_msgs/Pose pose
""" + DEPENDENCIES + """================================================================================
MSG: geometry_msgs/Pose
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
"""
IMAGE = HEADER + """uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
""" + DEPENDENCIES
