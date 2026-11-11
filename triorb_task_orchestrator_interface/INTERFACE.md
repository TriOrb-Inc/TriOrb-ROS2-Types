# triorb_task_orchestrator_interface 外部仕様

## TaskObservation / TaskTransition

`TaskObservation`は現在の状態を表し、状態変化時とheartbeatで発行する。
`TaskTransition`は走行開始・タスク終端の事実を一度通知する。
heartbeatで過去の遷移を再発行しない。

### 共通フィールド

| フィールド | 型 | 意味 |
|---|---|---|
| `stamp` | `builtin_interfaces/Time` | 状態評価時のROS時刻。同じ評価の観測と遷移は同じ値を使う |
| `source_session` | `string<=64` | 発行元のプロセス起動ごとに生成するUUID。空文字は禁止 |
| `observation_sequence` | `uint64` | 同じsession内の観測順序。1から開始し、heartbeatを含む各観測の発行で1増やす |
| `task_generation` | `uint64` | 同じsession内のタスク受付世代。受付ごとに1増やす。初回受付前は0。IDLEでも最後の値を維持する |
| `has_action` | `bool` | アクションの種別・実行状態を保持している場合にtrue |
| `action_type_id` | `uint16` | `RobotAction.action_type_id`と同じ値。`has_action=false`のとき0 |
| `action_phase` | `uint8` | `ActionExecutionReportDetail.PHASE_*`。`has_action=false`のとき`PHASE_UNKNOWN` |
| `reason_code` | `uint32` | `ActionExecutionReportDetail.REASON_*`。該当する理由がなければ`REASON_NONE` |

受信側は`source_session`と`observation_sequence`で観測の順序を判定し、
重複や逆順の観測では最終更新時刻を更新しない。
異なるsession間でsequenceやgenerationの大小を比較しない。
`stamp`の許容年齢や未来時刻の扱いは利用側の設定に従う。

### TaskObservation固有フィールド

`task_phase`は`uint8`で、次の定数を使う。

| 定数 | 値 | 意味 |
|---|---|---|
| `PHASE_IDLE` | 0 | タスク未実行 |
| `PHASE_COMPILING` | 1 | 受け付けたタスクを実行用データに変換中 |
| `PHASE_RUNNING` | 2 | タスク実行中 |
| `PHASE_PAUSING` | 3 | 一時停止への移行中 |
| `PHASE_PAUSE_RESTART_STOPPING` | 4 | 再実行による一時停止のため、現在のアクションを停止中 |
| `PHASE_PAUSED` | 5 | 一時停止中 |
| `PHASE_FAILED_HOLD` | 6 | 失敗した状態を保持中 |
| `PHASE_CANCELLING` | 7 | タスクのキャンセル処理中 |

### TaskTransition固有フィールド

`kind`は`uint8`で、次の定数を使う。0は無効値とし、発行しない。

| 定数 | 値 | 意味 |
|---|---|---|
| `KIND_NAVIGATION_STARTED` | 1 | 走行アクションの開始 |
| `KIND_TASK_SUCCEEDED` | 2 | タスクの成功終了 |
| `KIND_TASK_FAILED` | 3 | タスクの失敗 |
| `KIND_TASK_CANCELLED` | 4 | タスクのキャンセル完了 |

遷移には、同じ評価で作成した`TaskObservation`のsession、sequence、generation、stampを入れる。
アクションと理由のフィールドには遷移対象の情報を入れる。
終端後の観測がIDLEでも、遷移では終了したアクションの情報を保持できる。
受信側はsession、sequence、kindの組を遷移の重複判定に使う。
この型の追加によって既存の`TaskExecutionState`、`TaskEvent`、実行Actionの契約は変更しない。
