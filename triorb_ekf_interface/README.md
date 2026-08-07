[../](../README.md)

# Package: triorb_ekf_interface

## triorb_ekf_interface Types

### triorb_ekf_interface/srv/SetPoseSource
```bash
# ==EKF外部姿勢入力源切替Service==
uint8 SOURCE_VSLAM=0
uint8 SOURCE_TAGSLAM=1
uint8 SOURCE_COLLAB_TF=2

uint8 source
---
bool success
string message
uint8 active_source
```

`source` と `active_source` は次の値を使います。

- `SOURCE_VSLAM(0)`: `vslam/rig_tf`
- `SOURCE_TAGSLAM(1)`: `tagslam/rig_tf`
- `SOURCE_COLLAB_TF(2)`: `collab/group_tf`
