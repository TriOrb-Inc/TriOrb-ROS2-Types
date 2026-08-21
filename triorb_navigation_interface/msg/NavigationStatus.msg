std_msgs/Header header

# 移動要求
triorb_drive_interface/TriorbSetPos3 target

# 現在pose
NavigationPose current_pose

# 状態
uint8 STAND_BY=0
uint8 NAVIGATE=1
uint8 PAUSE=2
uint8 SUCCESS=3
uint8 FAILED=4
uint8 LEAVE_GOAL=5
uint8 navigate_state

# 経過時間 [sec]
float32 elapsed_sec

# navigation result
NavigationResult navigation_result
