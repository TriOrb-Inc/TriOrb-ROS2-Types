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
uint16 LOST_FAILED=9
uint16 BLOCKED_BY_PERMIT=10

# 使える現在姿勢を得られない場合。なぜ得られないか(自己位置のロスト・ノードの停止・
# 通信の途絶・時刻同期の異常)までは判定しないため、原因の切り分けは
# NavigationCommand の current_pose の valid と header.stamp を見ること。
uint16 POSE_NOT_RECEIVED=11   # 一度も受信していない。構成の問題(localization 未起動、topic 名の相違など)
uint16 POSE_UNUSABLE=12       # 受信しているが使えない。valid=false、または stamp が許容範囲外(古い / 未来)

# path_planner/step が途切れた場合。上位ノードの異常終了や、
# goal を畳んだあとの終了指令の送信漏れを検知するためのもの。
uint16 STEP_TIMEOUT=13

uint16 REJECT=255
uint16 NONE=65535

uint32 request_id
uint8 state
uint16 result
float32 elapsed_sec
string message
