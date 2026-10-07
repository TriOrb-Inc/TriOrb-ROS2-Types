# triorb_aux_interface

補器の継続出力と単発音声を要求するROS 2型。
LED・LCD・ランプ・継続音をまとめた指令と、イベントごとの単発音声指令を表す。
応答は受付結果であり、点灯・表示・再生の完了を表さない。

## 型定義

| 型 | 内容 |
|---|---|
| [AuxCommandConstants](msg/AuxCommandConstants.msg) | LED表示・色・点滅速度、STOP/LOOP、意味付き音声の定数 |
| [AuxOutput](msg/AuxOutput.msg) | 継続出力の完全なsnapshot。LCDは最大４行、各行16 byte |
| [AuxSoundOnce](msg/AuxSoundOnce.msg) | 発生時刻、発行元の起動UUID、イベントID、音声、音量、有効期間 |
| [SetAuxOutput](srv/SetAuxOutput.srv) | 継続出力の受付。既定の指令は消灯・音声停止 |
| [PlayAuxSoundOnce](srv/PlayAuxSoundOnce.srv) | 単発音声の受付。同一起動UUID・イベントIDで重複を識別 |

音声の定数値は機器の音源番号と別の識別子。STARTUP/FINISHは参照実装の音源13/14に対応する。
機器の送信値への変換はbridgeが持つ。特にSTOP/LOOPの数値をそのまま送信しない。
両Serviceの結果コードは共通で、0=ACCEPTED、1=ALREADY_ACCEPTED、2=INVALID_REQUEST、
3=EXPIRED、4=NOT_CONNECTED、5=BUSY。既定応答は `accepted=false` / INVALID_REQUEST。
ALREADY_ACCEPTEDは単発音声だけで使い、受付済みの同じ内容に再投入せず返す。

## 実装側で検証する制約

- LCDのASCII限定と行要素内の `||` 禁止、輝度0〜4095、音量0〜62、未対応の列挙値。
- 発行元UUIDは起動ごとに新規生成し、全ゼロは使わない。イベントIDはその起動内で一意にする。
- 有効期間は発生時刻からの正のDuration。受信後の残り時間は単調増加時計で管理する。
- 単発音声にNONEを指定しない。受付結果が不明な単発要求を自動再試行しない。

これらは型定義だけでは強制されない。音声の競合、重複記録の保持期間、継続要求の順序・受付期限、
時計差の扱いは実装時の設計事項であり、このパッケージはその動作を実装しない。
Behavior固有のイベント・状態型は今回追加していない。

## ビルド

```bash
colcon build --packages-select triorb_aux_interface
```

依存は `builtin_interfaces`、`unique_identifier_msgs`、`rosidl_default_generators`、`rosidl_default_runtime`。
