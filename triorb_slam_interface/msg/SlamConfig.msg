# Runtime keys published by vslam/config_publish.
uint8[] tracking_states
# One entry per camera: 0 is disabled and 1 is enabled.
uint8[] enable_camera
# -1 unknown, 0 fixed-map localization, 1 mapping.
int8 mapping_mode
string marker_mode
uint32 pub_freq
uint8 pub_quality
string map_path
bool pub_points
int32 kfrm_id_max
int32 kfrm_id_frez
# Mutable values accepted by vslam/signal.
# top_mask and bottom_mask are normalized ratios in the range 0.0 to 1.0.
float32 top_mask
float32 bottom_mask
# Additional implementation-specific values that can be represented without
# losing their ROS parameter type.
rcl_interfaces/Parameter[] parameters
