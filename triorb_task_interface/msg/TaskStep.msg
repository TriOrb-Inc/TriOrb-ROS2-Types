# task_orchestrator が組み立て、task_executor が実行する step 1 件。
#
# kind と target の語彙をこの msg に置いている。両ノードが別々の C++ 表を持つと
# 版ズレするため、契約は message 側の 1 箇所に集約する。
#
# 結果待ちの上限 (timeout) はここに載せない。B が parameter で持つ。
# 「そのデバイスがどれだけ待たせるか」はデバイス知識であり B の担当。
# 一方 settle_sec は値の選択が waypoint type 依存（relative の start_move だけ 0.3 s、
# 他の start_move は aux_event_delay_sec）なので、type を知る A が決めて渡す。

# ---- step の種類 ----
uint8 KIND_PUBLISH_STRING=0      # 文字列を 1 つ publish して即完了（aux event / policy / map パス など）
uint8 KIND_PUBLISH_NUMBER=1      # 数値を 1 つ publish して即完了
uint8 KIND_DELAY=2               # number 秒待つ
uint8 KIND_WAIT_EVENT=3          # action/event に text が来るまで待つ
uint8 KIND_MOVE=4                # automove の TriorbSetPos action
uint8 KIND_RELATIVE_MOVE=5       # drive/run_pos + 結果待ち
uint8 KIND_AWAIT_LIFTER=6        # lifter 指令 + 結果待ち
uint8 KIND_AWAIT_MAP_CHANGE=7    # map 切替 + 完了待ち（skip 要求 / 見積り延長を含む）
uint8 KIND_AWAIT_TAGSLAM_MAP=8   # TagSLAM map 切替 + 結果待ち（request_id 照合）
uint8 KIND_SERVICE_TRIGGER=9     # std_srvs/Trigger
uint8 KIND_SERVICE_SET_BOOL=10   # std_srvs/SetBool
uint8 KIND_SERVICE_AUTO_GAIN=11  # triorb_camera_argus/AutoGainTarget
uint8 kind

# ---- endpoint の選択。意味は kind ごとに決まる ----
#
# **番号は定数で持つ。コメントだけにしないこと。**
# 以前ここが `#` コメントだけだったため、A は独自の constexpr 13 個を、
# B は vector の並び順を、B のテストはもう一度並び順を持っていた。
# target を 1 つ挿入すると B が黙って別トピックへ publish する状態だった。
#
# 追加するときはこのブロックに定数を足し、両ノードでその名前を使う。

# KIND_PUBLISH_STRING の宛先
uint8 TARGET_STRING_AUX_EVENT=0        # aux/event
uint8 TARGET_STRING_COLLAB_AUX_EVENT=1 # collab/aux/event
uint8 TARGET_STRING_ACTION_EVENT=2     # action/event
uint8 TARGET_STRING_SNR_MUX=3          # snr_mux (topic 名は B の parameter)
uint8 TARGET_STRING_COLLAB_POLICY=4    # collab/navigation_policy
# map パスの publish は AWAIT_MAP_CHANGE / AWAIT_TAGSLAM_MAP の中で行うので
# ここには含めない。

# KIND_PUBLISH_NUMBER の宛先
uint8 TARGET_NUMBER_LOST_TIMEOUT=0     # drive/set_lost_timeout_sec (float32)
uint8 TARGET_NUMBER_ERROR_RESET=1      # triorb/error/reset (uint8)
uint8 TARGET_NUMBER_POSITIONING_MODE=2 # drive/set_positioning_timeout_mode (uint8)

# KIND_MOVE の測位源。**triorb_drive_interface/TriorbRunSetting の TF_SOURCE_* と
# 同じ値でなければならない。** B 側に static_assert がある。
uint8 TARGET_MOVE_UNSPECIFIED=0
uint8 TARGET_MOVE_VSLAM=1
uint8 TARGET_MOVE_TAGSLAM=2
uint8 TARGET_MOVE_COLLAB=3             # collab は action 未定のため topic publish

# KIND_AWAIT_LIFTER の宛先
uint8 TARGET_LIFTER_DRIVE=0            # drive/run_lifter
uint8 TARGET_LIFTER_COLLAB=1           # collab/run_lifter

# KIND_SERVICE_TRIGGER の宛先
uint8 TARGET_TRIGGER_TAGSLAM_STOP=0    # tagslam/stop
uint8 TARGET_TRIGGER_TAGSLAM_RESUME=1  # tagslam/resume
uint8 TARGET_TRIGGER_VSLAM_STOP=2      # vslam/stop
uint8 TARGET_TRIGGER_VSLAM_RESUME=3    # vslam/resume

uint8 target

# ---- payload ----
string name                 # feedback / log 用の表示名（例 "MoveStep(P1)"）
string text                 # 文字列 payload（event 名 / lifter 指令 / map パス / JSON）
float64 number              # 数値 payload（DELAY の秒数 / float 値 / uint8 値）
bool flag                   # bool payload（SLS の on/off）
float64 settle_sec          # 完了後に待つ秒数。0 = 待たない

# ---- 移動系のみ使用（KIND_MOVE / KIND_RELATIVE_MOVE）----
# 他の kind では未使用。tagged union の代償だが、送信頻度が低いので実害はない。
triorb_drive_interface/TriorbSetPos3 pose
