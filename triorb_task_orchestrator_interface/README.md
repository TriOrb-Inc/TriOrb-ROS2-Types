# triorb_task_orchestrator_interface

## Navigation の座標変換失敗の定数名移行

[Issue #77](https://github.com/TriOrb-Inc/TriOrb-ROS2-Types/issues/77) の対応です。

`ActionExecutionReportDetail` の正名は
`REASON_NAVIGATION_TRANSFORM_FAILED = 9` です。
旧名 `REASON_NAVIGATION_TRANSFORM_FAILE = 9` も非推奨の互換別名として残します。
この変更では旧名を削除せず、値9の意味、message field、Action の入出力は変更しません。

新しいコードでは正名を使用してください。C++ / Python 等の生成 API で正名を使うには、
更新した ROS Types と利用側を再ビルドします。旧名を参照する既存クライアントは引き続きビルドでき、
送受信される `ActionExecutionReport.reason_code` は両名とも9です。
外部クライアントの移行完了は確認できていないため、旧名の削除時期は未定です。

### 移行前の workspace 内の参照調査

- Types: `msg/ActionExecutionReportDetail.msg` の定義。
- Navigation Subsystems / `triorb_action_executor`: `src/handler/navigation_action_handler.cpp`
  と `docs/navigation_action_executor_state.md` を正名に移行。
- 同 module の `test/task_orchestrator_integration_test_report.md` の B07 記録3箇所は、
  当時の出力を記録した旧名のため維持し、移行注記を追加。
- Task Orchestrator 本体と GUI を含む workspace の他のコードには旧名の参照なし。
  Orchestrator の失敗判定は ROS Result code と `phase` / `result` に基づくため、実装変更は不要。
- workspace 外の外部クライアントは未確認。互換別名で既存の生成 API 利用を維持する。

### 検証

ROS 2 Humble で型生成・Action Executor / Task Orchestrator のビルドと、次の回帰を確認しました。

- 生成 C++ / Python API で新旧定数がどちらも9であること。
- Navigation の `TRANSFORM_FAILED` が `ABORTED` / `PHASE_FAILED` /
  `RESULT_FAILED` / `reason_code=9` として Action Executor から返ること。
- Orchestrator がこの失敗を `FAILED_HOLD` に保持し、明示的な RESUME まで再実行しないこと。

Action Executor 55件、Task Compiler 44件、Task Orchestrator 82件、
interface package の CMake/XML lint が成功しました。
Python では新旧定数による report のシリアライズ結果の一致と復元も確認しました。
Node 試験は固定 Profile と fake 下位を使用し、実機試験は未実施です。
