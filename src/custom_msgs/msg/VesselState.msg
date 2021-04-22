# This message describes the internal state of the vessel simulator

geometry_msgs/Pose pose # NED convention
geometry_msgs/Twist velocity # NED/BODY convention
string vessel_frame_id # Frame ID of the BODY frame
float64 w_1 # propeller 1 rpm
float64 w_2 # propeller 2 rpm
float64 a_1 # azimuth angle 1 (radians)
float64 a_2 # azimuth angle 2
