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

#==ロボット → FMS: 現在位置(2 Hz)==
# navigation/current_pose(NavigationPose)の写し
builtin_interfaces/Time stamp
string frame_id

# 位置の出どころ。NavigationPose.source(TF_SOURCE_*)を文字列にしたもの
string SOURCE_UNSPECIFIED="unspecified"  # 自己位置が成立していない
string SOURCE_VSLAM="vslam"
string SOURCE_TAGSLAM="tagslam"
string SOURCE_COLLAB="collab"            # 協調搬送の親機の位置から求めた値
string source                            # SOURCE_*

float64 x                                # [m]
float64 y                                # [m]
float64 deg                              # [deg]
bool valid                               # false なら x / y / deg は無効
