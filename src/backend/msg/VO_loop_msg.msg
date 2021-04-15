std_msgs/Header header
geometry_msgs/Pose pose

std_msgs/Bool is_keyframe
int32 keyframe_id

int32 landmark_size
backend/IDPoint3D_msg[] landmarks
backend/IDPoint2D_msg[] features

std_msgs/Bool loop_found
int32 match_id
geometry_msgs/Pose pose_loop
