std_msgs/Header header   # この姿勢の時刻と frame_i

# TF source used for localization
uint32 TF_SOURCE_UNSPECIFIED=0
uint32 TF_SOURCE_VSLAM=1
uint32 TF_SOURCE_TAGSLAM=2
uint32 TF_SOURCE_COLLAB=3
uint32 source

float64 x
float64 y
float64 deg
bool valid #現在姿勢が取れているか

uint8 CONFIDENCE_LOW=0
uint8 CONFIDENCE_MIDDLE=1
uint8 CONFIDENCE_HIGH =2
uint8 confidence
