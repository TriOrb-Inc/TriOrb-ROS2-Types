# 位置決め停滞/振動検出を無効化し、従来通り通常の移動終了判定だけを使う。
uint8 DISABLED=0
# 位置決め停滞/振動検出時に移動失敗として扱い、ERROR通知を発報する。
uint8 FAIL=1
# 位置決め停滞/振動検出時に到達困難だが許容完了として扱い、WARN通知を発報する。
uint8 SUCCESS=2

# navigation_managerからNavigatorへ配信する現在の位置決めタイムアウト動作モード。
uint8 mode
