std_msgs/Header header

uint8 STAND_BY=0
uint8 NAVIGATE=1
uint8 PAUSE=2
uint8 SUCCESS=3
uint8 FAILED=4
uint8 LEAVE_GOAL=5

uint16 TIMEOUT_FAILED=0
uint16 HALF_TIMEOUT=1
uint16 TRANSFORM_FAILED=2
uint16 NO_CHANGE_TIMESTAMP=3
uint16 FORCE_STOP=4
uint16 NAVIGATION_FAILED=5
uint16 NAVIGATION_SUCCESS=6
uint16 PROGRESS=7
uint16 FORCE_SUCCESS=8
uint16 BLOCKED_BY_PERMIT=10
uint16 POSE_NOT_RECEIVED=11   # 使える現在姿勢を一度も受信しないまま timeout
uint16 POSE_UNUSABLE=12       # 受信した現在姿勢が使えない状態のまま timeout
uint16 STEP_TIMEOUT=13        # path_planner/step が途切れた
uint16 REJECT=255
uint16 NONE=65535

uint32 request_id
uint8 state
uint16 result
float32 elapsed_sec
string message
