#**
#* Copyright 2023 TriOrb Inc.
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

#==障害物マップを利用した経路生成(when param=0, default value is used)==
TriorbPos3 start          # start position -> 現在位置を取得すればよい？
TriorbPos3 goal           # Goal position
float32 robot_w           # robot width  ( luggage width )  (default=0.5)
float32 robot_h           # robot height ( luggage height ) (default=0.5)
uint8 down_sample_rate    # image reduction size   (default=1)
float32 xy_reso           # m/px in obstacle map   (default=0.01)
float32 theta_reso        # minimum rotation [deg] (default=360/64)
float32 lam               # param (default=1.0)
float32 r                 # param (default=1.0/6.5)
float32 epsilon           # param (default=0.05)
