# triorb_task_orchestrator_interface

タスクの実行要求、実行状態、アクションの制御・報告を受け渡すROS 2型。
タスク状態の継続的な観測と、一度だけ通知する遷移を別のメッセージとして提供する。
タスクの実行処理や、表示・音声の選択処理は含まない。

## 観測用の型

| 型 | 内容 |
|---|---|
| [TaskObservation](msg/TaskObservation.msg) | 発行元、順序、タスク世代を伴う状態snapshot |
| [TaskTransition](msg/TaskTransition.msg) | 走行開始、成功、失敗、キャンセルの遷移 |

型の契約は[INTERFACE.md](INTERFACE.md)を参照。
既存の実行要求・報告の型は、各`msg`、`srv`、`action`ファイルに定義する。

## ビルド

ROS 2を読み込んだworkspaceで実行する。

```bash
colcon build --packages-up-to triorb_task_orchestrator_interface
```

`action_msgs`、`builtin_interfaces`、`triorb_drive_interface`、
`rosidl_default_generators`、`rosidl_default_runtime`に依存する。
検証内容と未実施事項は[test/README.md](test/README.md)を参照。
