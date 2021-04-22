# Custom message for navigation estimate
uint8 NO_POSITION=0
uint8 AUTONOMOUS_POSITION=1
uint8 DIFFERENTIAL_POSITION=2
uint8 FIXED_RTK=4
uint8 FLOATING_RTK=5

std_msgs/Header header
string child_frame_id
geometry_msgs/Pose pose
geometry_msgs/Twist twist
uint8 gps_status
