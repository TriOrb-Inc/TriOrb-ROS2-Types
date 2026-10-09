# triorb_snr_mux_interface

SNR-MUXの受信フレーム、通信状態、補器出力の要求を表すROS 2型。
UART接続やIMU値の解釈を実装せず、受信データと通信状態の受け渡しに使う。
IMU出力と診断には既存の `sensor_msgs/msg/Imu` と `diagnostic_msgs/msg/DiagnosticArray` を利用する。

## 型定義

| 型 | 内容 |
|---|---|
| [SnrMuxFrame](msg/SnrMuxFrame.msg) | 受信完了時刻、プロセス内の接続世代、ID、SEQ、最大255 byteのpayload |
| [SnrMuxStatus](msg/SnrMuxStatus.msg) | 状態の作成時刻、接続状態６種、受信有無・最終受信時刻・途絶 |

受信payloadはヘッダ・CRCを含まない。IMUの36 byte制約や数値検証は利用側が行う。
ポート接続と正常フレームの受信状態は独立して扱う。
最終受信時刻は受信有無のflagと組み合わせ、ROS時刻0を未受信の印として使わない。
`SnrMuxFrame`の接続世代はbridgeの再起動で同じ値を取り得るため、この型だけでは再起動を確実に識別できない。

## 補器出力

| 型 | 内容 |
|---|---|
| [AuxOutput](msg/AuxOutput.msg) | 生成時刻、LED、LCD、継続音、ランプの完全snapshot |
| [AuxSoundOnce](msg/AuxSoundOnce.msg) | 生成時刻、session、event ID、音声cue、音量、有効期間 |
| [SetAuxOutput](srv/SetAuxOutput.srv) | 継続出力の受付要求と受付結果 |
| [PlayAuxSoundOnce](srv/PlayAuxSoundOnce.srv) | 単発音声の受付要求と受付結果 |

Responseの`accepted`は保存・queueへの受付を表し、機器反映や再生完了を表さない。
結果コードはACCEPTED=0、ALREADY_ACCEPTED=1、INVALID_REQUEST=2、EXPIRED=3、NOT_CONNECTED=4、BUSY=5。
0と1だけがaccepted=true。理由文は人向けで、Clientはresult_codeで分岐する。

音声cueの定数は`AuxOutput.CUE_*`を共用し、機体の音源番号との対応はServer側で設定する。
継続音はLOOP / STOP、単発音は別Serviceを使う。
Clientはheartbeatごとにstampを更新し、同じsession内でevent_idを再利用しない。
単発の無応答を再生失敗と決め付けず、自動再試行しない。

LCDは最大4行・各16 byteの印字可能ASCIIとし、区切り込み64 byte以下。
行内の`||`と最終行以外の末尾`|`を含めない。
Serverが行間へ区切りを付け、表現できない文字を置換・切り詰めて受理しない。

補器用の型もこのパッケージに統合し、別パッケージには分けない。

## ビルド

ROS 2を読み込んだworkspaceで実行する。型生成とService送受信はJazzyで検証する。

```bash
colcon build --packages-select triorb_snr_mux_interface
```

## 依存関係

`builtin_interfaces`、`rosidl_default_generators`、`rosidl_default_runtime`。
