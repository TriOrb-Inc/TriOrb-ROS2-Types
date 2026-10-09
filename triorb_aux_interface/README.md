# triorb_aux_interface

## 概要

補器の継続出力と単発音声を要求するROS 2のメッセージ・Service型。
機器のUART形式やCRCを呼び出し側へ公開せず、意味付きの出力を渡す。

## 提供する型

| 型 | 内容 |
|---|---|
| `msg/AuxOutput` | 生成時刻、LED、LCD、継続音、ランプの完全snapshot |
| `msg/AuxSoundOnce` | 生成時刻、session、event ID、音声cue、音量、有効期間 |
| `srv/SetAuxOutput` | 継続出力の受付要求と受付結果 |
| `srv/PlayAuxSoundOnce` | 単発音声の受付要求と受付結果 |

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

## ビルドと検証

```bash
colcon build --packages-select triorb_aux_interface
```

2026-10-09にROS 2 Jazzyで型生成・ビルドを確認。
Serviceの送受信と結果コードは利用ノードの擬似UARTテストで検証する。
