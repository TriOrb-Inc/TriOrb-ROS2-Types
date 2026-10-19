[../](../README.md)

# Package: triorb_ekf_interface

## triorb_ekf_interface Types

### triorb_ekf_interface/srv/SetPoseSource
```bash
# ==EKF外部姿勢入力源切替Service==
uint8 SOURCE_LICAMSLAM=1
uint8 SOURCE_TAGSLAM=2
uint8 SOURCE_COLLAB_TF=3

uint8 source
---
bool success
string message
uint8 active_source
```

`source` と `active_source` は次の値を使います。

- `SOURCE_LICAMSLAM(1)`: LicamSLAMの`vslam/rig_tf`
- `SOURCE_TAGSLAM(2)`: TagSLAMの`tagslam/rig_tf`
- `SOURCE_COLLAB_TF(3)`: 協調動作グループの`collab/group_tf`

`SOURCE_VSLAM`は`SOURCE_LICAMSLAM`へ改名しました。数値は`1`のままです。
利用側の定数参照を更新し、このパッケージと利用側パッケージを再ビルドしてください。
Topic名、要求・応答のフィールド名と型は変更していません。
