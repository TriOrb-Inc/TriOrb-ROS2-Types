# DRAFT: complete runtime configuration exposed by the backend.
bool running
# One entry per camera: 0 means disabled and 1 means enabled.
uint8[] enabled_cameras
string current_map
# -1 unknown, 0 fixed-map localization, 1 mapping.
int8 mapping_mode
float32 top_mask
float32 bottom_mask
float32 image_update_rate_hz
uint8 image_quality
# Additional implementation-specific settings remain typed ROS parameters.
rcl_interfaces/Parameter[] parameters
