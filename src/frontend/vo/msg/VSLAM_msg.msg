std_msgs/Header header
geometry_msgs/Pose pose

std_msgs/Bool is_keyframe
int32 keyframe_id

int32 landmark_size
vo/IDPoint3D_msg[] landmarks
vo/IDPoint2D_msg[] lfeatures
vo/IDPoint2D_msg[] rfeatures

std_msgs/Bool loop_found
int32 match_id
geometry_msgs/Pose pose_loop
