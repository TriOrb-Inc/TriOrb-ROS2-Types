std_msgs/Header header

triorb_drive_interface/TriorbSetPos3 target
NavigationPose current_pose

uint8 STAND_BY=0
uint8 NAVIGATE=1
uint8 PAUSE=2
uint8 SUCCESS=3
uint8 FAILED=4
uint8 LEAVE_GOAL=5
uint8 navigate_state

float32 elapsed_sec
NavigationResult navigation_result
