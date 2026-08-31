<<<<<<< HEAD
# route CSV の 1 行を message で表したもの。
#
# ExecuteRoute の goal で外部クライアントから送られる。triorb_task_orchestrator はこれを
# route_directory 配下の一時 CSV へ書き戻してから、既存の CSV 経路
# （type の正規化 / 値の検証 / include_csv の展開）に載せる。
#
# 列は route CSV と 1:1 で対応する。**value が CSV の action 列**。
# triorb-route-sequence JSON（convert_csv2json.py の出力）の 1 要素に相当する。

string type      # waypoint type。日本語表記も可（"地図切替" / "switch_map" など）
string name      # 表示名。route marker は "route"、include_csv は "include_csv"
string value     # CSV の action 列。pause の秒数、map パス、lifter 指令など type ごとに意味が変わる

# ---- 移動系 (point / into / relative / collab_move) だけが使う ----
#
# **未指定は 0 でよい。** CSV の空セルと同じ扱いになる。
# ただし acc_xy / acc_w が 0 のままだと automove が goal を REJECT するため、
# 移動系では必ず指定すること（route 開始時に RouteError で弾かれる）。

float64 x        # 目標位置 [m]
float64 y        # 目標位置 [m]
float64 w        # 目標姿勢 [deg]

float64 vxy      # 並進速度 [m/s]。goal の ratio_speed が掛かる
float64 vw       # 回転速度 [rad/s]。goal の ratio_speed が掛かる

float64 acc_xy   # 位置の許容誤差 [m]。goal の bias_accuracy_xy が足される
float64 acc_w    # 姿勢の許容誤差 [deg]。goal の bias_accuracy_deg が足される

uint8 gain       # TriorbRunSetting/gain_no (not set:0, basic:1)
uint32 accel_ms  # 加減速時間 [ms]。**0 は未指定**として扱い、既定の 1000 が使われる
string camera    # "1|0|1|1" 形式。0 の位置のカメラを測位から外す。空なら全て使う
=======
# Row type in the triorb-route-sequence representation.
string type
string name
string value
>>>>>>> dev/v2.0.0
