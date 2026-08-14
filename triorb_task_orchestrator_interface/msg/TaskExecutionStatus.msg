# triorb_task_orchestrator の route 実行状態。
#
# ExecuteRoute の feedback と GetTaskExecutionStatus の応答で返す。
# feedback は current_sequence_index / current_loop_index / current_state の
# いずれかが変わったときだけ流れる（step 単位の進行は triorb_task_orchestrator のログにしか出ない）。
#
# triorb_task_orchestrator は状態を自分で決めない。Running / Paused / FailureHold は
# triorb_task_executor の runner_state の写しで、triorb_task_orchestrator が決めるのは
# STATE_STOPPING と STATE_FINISHED だけ。

# ---- current_state の語彙 ----
#
# **文字列は定数で持つ。コメントだけにしないこと。**
# triorb_task_orchestrator の routeStateName() と GUI の表示条件がそれぞれ独自の文字列表を持つと、
# 綴りを 1 つ変えただけで GUI が黙って状態を見失う。契約はここに集約する。
#
# 遷移の全体像は triorb_task_orchestrator/DESIGN_TWO_NODE.md を見ること。

string STATE_IDLE=Idle            # route を持っていない。triorb_task_executor が CANCELED を返した直後もここ
string STATE_RUNNING=Running      # 実行中。triorb_task_executor の runner が Running
string STATE_PAUSED=Paused        # 一時停止中。triorb_task_executor の runner が Paused
string STATE_FAILURE_HOLD=FailureHold  # step 失敗で保持中。**goal は閉じない**。resume / exit 待ち
string STATE_STOPPING=Stopping    # exit / cancel を受けて triorb_task_executor の決着待ち。triorb_task_orchestrator が決める
string STATE_FINISHED=Finished    # 全 sequence x loop を消化した。triorb_task_orchestrator が決める

# 上記 6 つのいずれか。**CSV の読み込み・展開に失敗した場合はここに出ない。**
# その場合は route が始まらないので STATE_IDLE のままで、原因は
# ExecuteRoute result の final_status="RouteError" と message に出る。
string current_state

# ---- 進行 ----
#
# 総数と現在位置の対を 2 組持つ。名前を total_* / current_*_index で
# 揃えてあるので、どちらがどちらか読み違えないこと。

uint32 total_loops              # 周回する回数。goal の loop をそのまま返す
uint32 current_loop_index       # 今が何周目か。**1 始まり**（current_loop_index <= total_loops）

uint32 total_sequences          # 展開後の sequence 数。**1 周ぶん**。
                                # include_csv は展開済みなので CSV の行数とは一致しない
uint32 current_sequence_index   # 現在位置。**0 始まり**。周が変わると 0 に戻る。
                                # Finished 時は範囲外になるため total_sequences - 1 に丸める

# ---- 表示用 ----

string route_csv_name           # 実行中の route CSV 名。goal の sequence を書き出した
                                # 一時 CSV の名前（".tmp_<時刻>.csv"）が入る
string current_sequence_name    # 現在の sequence の表示名。waypoint の name、空なら type

# route 上の位置を index の列で表したもの。**幾何的な経路ではない。**
# include_csv のネストをドットで繋ぐ。
# 例 "0.3.1" = 先頭要素 -> その中の 4 番目 -> さらにその中の 2 番目
string current_index_path

# current_index_path のネスト段数（ドット数 + 1）。
# トップレベルは 1。route が空のときだけ 0
uint32 current_include_depth
