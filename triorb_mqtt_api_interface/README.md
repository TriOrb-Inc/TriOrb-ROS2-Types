[../](../README.md)

# Package: triorb_mqtt_api_interface

FMS(Fleet Management System)とロボットの間の MQTT 通信で使う型。ロボット側の `triorb_mqtt_ros_bridge` と FMS 側のアダプターが同じ定義を見る。
MQTT のトピックは `"<prefix>/" + MqttTopics の定数`、payload は各 msg を JSON にしたもの(キー = フィールド名、`builtin_interfaces/Time` は `{"sec", "nanosec"}`)。トピックごとの周期・QoS・retain は `triorb_mqtt_ros_bridge/INTERFACE.md` を参照。

## triorb_mqtt_api_interface Types

### triorb_mqtt_api_interface/msg/MqttTopics
```bash
#==両方向: MQTT トピック名の定数==
# 実際のトピックは "<prefix>/" + 定数。<prefix> はロボットの識別子(/ を含まない 1 階層)
# ロボット → FMS
string STATE="mqtt_ros_bridge/state"                        # RobotState
string POSE="mqtt_ros_bridge/pose"                          # RobotPose
string TASK_STATE="mqtt_ros_bridge/task/state"              # TaskState
string STATE_RESPONSE="mqtt_ros_bridge/state/response"      # DiagnosticsSnapshot
string HEARTBEAT="mqtt_ros_bridge/heartbeat"                # Heartbeat(retain)
string COMMAND_RESULT="mqtt_ros_bridge/cmd/result"          # CommandResult
string PONG="mqtt_ros_bridge/pong"                          # 素の文字列(PING の折り返し)
# FMS → ロボット
string CMD_TASK_EXECUTE="mqtt_ros_bridge/cmd/task/execute"  # TaskExecuteCommand
string CMD_TASK_CONTROL="mqtt_ros_bridge/cmd/task/control"  # TaskControlCommand
string CMD_STOP="mqtt_ros_bridge/cmd/stop"                  # StopCommand
string STATE_REQUEST="mqtt_ros_bridge/state/request"        # DiagnosticsRequest
string PING="mqtt_ros_bridge/ping"                          # 素の文字列 "<送り主>:<エポックミリ秒>"
```

### triorb_mqtt_api_interface/msg/RobotState
```bash
#==ロボット → FMS: ロボットの状態(1 Hz)==
# FMS が統一状態コードを導出するための値。統一状態コードそのものは載せない
builtin_interfaces/Time stamp

# ECU の状態(robot/ecu_status)
bool facts_fresh              # ecu_status が 3 秒以内に取れている。false なら下の bool は無効
bool emergency_stop           # 非常停止中
bool moving                   # 走行中
bool motor_energized          # 励磁中
bool brake_free               # 電磁ブレーキ解放

# タスク
bool navigating               # タスク実行中
string active_task_file       # 実行中のタスクファイル名。無ければ空

# 電源
float32 voltage               # ECU の電源電圧 [V](robot/ecu_status)。未受信なら 0
bool charging                 # 充電中(battery/status)
```

### triorb_mqtt_api_interface/msg/RobotPose
```bash
#==ロボット → FMS: 現在位置(2 Hz)==
# navigation/current_pose(NavigationPose)の写し
builtin_interfaces/Time stamp
string frame_id

# 位置の出どころ。NavigationPose.source(TF_SOURCE_*)を文字列にしたもの
string SOURCE_UNSPECIFIED="unspecified"  # 自己位置が成立していない
string SOURCE_VSLAM="vslam"
string SOURCE_TAGSLAM="tagslam"
string SOURCE_COLLAB="collab"            # 協調搬送の親機の位置から求めた値
string source                            # SOURCE_*

float64 x                                # [m]
float64 y                                # [m]
float64 deg                              # [deg]
bool valid                               # false なら x / y / deg は無効
```

### triorb_mqtt_api_interface/msg/TaskState
```bash
#==ロボット → FMS: タスク実行の状態(変化時 + 実行中は 1 Hz)==
# TaskExecutionState の写しに最終結果を足したもの
builtin_interfaces/Time stamp
string STATE_RUNNING="running"  # 実行中
string STATE_PAUSED="paused"    # 一時停止中
string STATE_DONE="done"        # 終了。final_status に結果
string state                    # STATE_*。Bridge が action の受理・pause / resume の成否・result から決める
string orchestrator_state       # TaskExecutionState.current_state をそのまま(参考)
uint32 loop                     # 周回数
uint32 current_loop_index       # 現在の周回
uint32 action_instance_id       # 実行中の action の順番
string file_name

string FINAL_FINISHED="Finished"
string FINAL_ERROR="Error"
string FINAL_STOPPED="Stopped"
string FINAL_CANCELED="Canceled"
string final_status             # 最終結果(FINAL_*)。実行中は空
string message                  # 失敗・停止の理由
```

### triorb_mqtt_api_interface/msg/Heartbeat
```bash
#==ロボット → FMS: ハートビート(2 秒、retain)==
# FMS が接続中のロボットを見つけ、prefix の重複を検出するために使う。切断時は Last Will が retain を空 payload で消す
builtin_interfaces/Time stamp
string prefix                 # MQTT の <prefix>
string hostname
string ip                     # 代表 IPv4
```

### triorb_mqtt_api_interface/msg/DiagnosticsRequest
```bash
#==FMS → ロボット: 診断の全文の要求==
string request_id             # FMS が一意に振る
string prefix                 # 診断名の前方一致。空なら全件
```

### triorb_mqtt_api_interface/msg/DiagnosticsSnapshot
```bash
#==ロボット → FMS: 診断の全文(DiagnosticsRequest への応答)==
# robot/state/get の応答(DiagnosticArray)の header.stamp と status[] を写す。header.frame_id は使わないので載せない
string request_id              # 要求の request_id
builtin_interfaces/Time stamp  # 集約結果の header.stamp
diagnostic_msgs/DiagnosticStatus[] status
```

### triorb_mqtt_api_interface/msg/CommandResult
```bash
#==ロボット → FMS: コマンドへの応答==
# 1 コマンドにつき、受理時(accepted)と完了時(succeeded / failed)に出る。拒否は rejected の 1 通だけ。
# stop は accepted の後、停止を観測したら confirmed、観測できなければ failed
builtin_interfaces/Time stamp
string request_id                   # コマンドの request_id
string PHASE_ACCEPTED="accepted"    # 受理した。以後 succeeded / failed / confirmed のどれかが来る
string PHASE_REJECTED="rejected"    # 検証で拒否した。これで終わり
string PHASE_SUCCEEDED="succeeded"  # 完了。ROS 2 側の呼び出しが成功した
string PHASE_FAILED="failed"        # 完了。失敗または deadline 超過
string PHASE_CONFIRMED="confirmed"  # 完了。stop のみ。停止を観測した
string phase                        # PHASE_*
string message                      # 拒否・失敗の理由
```

### triorb_mqtt_api_interface/msg/TaskExecuteCommand
```bash
#==FMS → ロボット: タスク実行==
string request_id             # FMS が一意に振る
string file_name              # 保存済みタスクファイル名
uint32 loop                   # 周回数(1 以上)
float32 ratio_speed           # 速度倍率(0 より大)
float32 bias_accuracy_xy      # [m]
float32 bias_accuracy_deg     # [deg]
```

### triorb_mqtt_api_interface/msg/TaskControlCommand
```bash
#==FMS → ロボット: 実行中タスクの制御==
string request_id             # FMS が一意に振る
string COMMAND_PAUSE="pause"
string COMMAND_RESUME="resume"
string COMMAND_STOP="stop"
string command                # COMMAND_*
```

### triorb_mqtt_api_interface/msg/StopCommand
```bash
#==FMS → ロボット: 走行停止(タスクの有無に関係なく車輪を止める)==
string request_id             # FMS が一意に振る
```
