#**
#* Copyright 2026 TriOrb Inc.
#*
#* Licensed under the Apache License, Version 2.0 (the "License");
#* you may not use this file except in compliance with the License.
#* You may obtain a copy of the License at
#*
#*     http://www.apache.org/licenses/LICENSE-2.0
#*
#* Unless required by applicable law or agreed to in writing, software
#* distributed under the License is distributed on an "AS IS" BASIS,
#* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#* See the License for the specific language governing permissions and
#* limitations under the License.
#**

#==走行方向・並進速度・角速度==

float32 direction            # 走行方向[deg],range [-180,180] 並進速度が0の場合、走行方向に意味は無い
float32 translational_speed  # 並進速度[m/s] > 0
float32 angular_velocity     # 角速度[rad/s]

# 座標定義
# ロボット座標系の+Y軸(前面方向)を0degとし時計回りを正とする