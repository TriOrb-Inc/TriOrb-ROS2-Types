# triorb_task_orchestrator_interface 検証記録

## 対象

`TaskObservation`と`TaskTransition`のROS 2型生成と送受信を確認する。
状態判定、発行周期、重複排除は利用側の処理として検証する。

## 実行方法

ROS 2環境を読み込んだworkspaceで実行する。

```bash
colcon build --packages-up-to triorb_task_orchestrator_interface
source install/setup.bash
colcon test --packages-select triorb_task_orchestrator_interface
colcon test-result --verbose
```

## 実施結果

2026-11-10、Ubuntu 22.04、ROS 2 Humble、Python 3.10.12で実施。
通常の作業ディレクトリの`PR/aux-controller-ros2-types`ブランチから再ビルドし、
`/tmp/triorb-aux-types-review-build/install`に生成した型を使用した。

| 確認内容 | 結果 |
|---|---|
| 依存する`triorb_drive_interface`と本パッケージのビルド | 成功。C、C++、Pythonの型とtypesupportを生成 |
| `ament_lint_cmake` / `ament_xmllint` | 成功 |
| `ament_copyright` | 終了コード0。ただし対象ファイルを検出せず0件。追加msgのライセンス表記と、V2.0.0の規約に従う2023年表記は目視確認 |
| Pythonの新2型のimport | 成功 |
| `rclpy.serialization`によるCDR往復変換 | 成功。`uint16`、`uint32`、`uint64`の最大値を保持 |
| 65文字の`source_session`指定 | 生成Python型が拒否することを確認 |
| 実ROS 2 topicによる新2型の送受信 | 成功。localhostの独立domainで2ノードを作成し、受信値と送信値の一致を確認 |
| 生成型を使用したaux_controllerの既存単体テスト | `test_aux_controller_node.py`の7件が成功。ROS入出力と入力から出力までの処理を確認 |
| `git diff --check` | 成功 |

型の送受信は手動の実行確認として行った。
型定義を再記述する単体テストは追加していない。

## 未実施

スタブテスト、実機テスト、リリースタグを用いた結合テストは未実施。
机上のJetsonと実機への接続環境がなく、リリースタグの実機導入も行っていないため。
本パッケージの検証では実際のタスク状態遷移や補器出力は実行していない。

## 2026-11-11: PR前の型生成と利用側の連携確認

独立workspaceへコピーして実IDLからbuildし、`colcon test` / `colcon test-result`がerrors / failures / skippedすべて0で終了した。
同じ生成型を使用して、実Task Orchestratorとaux_controllerを別プロセスで起動するホスト連携確認7件が成功した。
下位Executor・補器Service・機器入力はテスト側で模擬している。
型生成と連携確認のworkspaceは`/home/kuma/AMR-System-dev-ws/worktrees/aux-navigation-integration-20261111/`、
型のPR前検証ログは`/home/kuma/AMR-System-dev-ws/worktrees/aux-controller-pr-20261111/results/types-test.log`と`types-test-result.log`。
机上Jetson・実機を使用していないため、スタブ・実機・リリースタグでの結合テストは引き続き未実施。
