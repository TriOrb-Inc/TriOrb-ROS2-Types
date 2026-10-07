# triorb_snr_mux_interface

SNR-MUXの受信フレームと通信状態を表すROS 2型。
UART接続やIMU値の解釈を実装せず、受信データと通信状態の受け渡しに使う。
IMU出力と診断には既存の `sensor_msgs/msg/Imu` と `diagnostic_msgs/msg/DiagnosticArray` を利用する。

## 型定義

| 型 | 内容 |
|---|---|
| [SnrMuxFrame](msg/SnrMuxFrame.msg) | 受信完了時刻、プロセス内の接続世代、ID、SEQ、最大255 byteのpayload |
| [SnrMuxStatus](msg/SnrMuxStatus.msg) | 接続状態６種、受信有無・時刻・途絶、累積エラー件数、人向けの理由 |

受信payloadはヘッダ・CRCを含まない。IMUの36 byte制約や数値検証は利用側が行う。
ポート接続と正常フレームの受信状態は独立して扱う。
最終受信時刻は受信有無のflagと組み合わせ、ROS時刻0を未受信の印として使わない。
接続世代はbridgeの再起動で同じ値を取り得るため、この型だけでは再起動を確実に識別できない。

## ビルドと検証

ROS 2 Humbleを読み込んだworkspaceで実行する。

```bash
colcon build --packages-select triorb_snr_mux_interface
source install/local_setup.bash
colcon test --packages-select triorb_snr_mux_interface --return-code-on-test-failure
colcon test-result --verbose
```

`test/test_serialization.py` は生成された型を使い、payload長0・36・255と256の拒否、
ID・SEQ・接続世代・受信時刻のシリアライズ、受信時刻0と未受信の区別を確認する。
通信ノード・parser本体や実機動作を検証するテストではない。

## 依存関係

`builtin_interfaces`、`rosidl_default_generators`、`rosidl_default_runtime`。
テストには `ament_cmake_pytest` と `rclpy` を使う。
