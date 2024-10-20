#**
#* Copyright 2023 TriOrb Inc.
#*
#* This program is free software; you can redistribute it and/or
#* modify it under the terms of the GNU Lesser General Public
#* License as published by the Free Software Foundation; either
#* version 3 of the License, or (at your option) any later version.
#* 
#* This program is distributed in the hope that it will be useful,
#* but WITHOUT ANY WARRANTY; without even the implied warranty of
#* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#* Lesser General Public License for more details.
#* 
#* You should have received a copy of the GNU Lesser General Public License
#* along with this program; if not, write to the Free Software Foundation,
#* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
#**

#==仕様：S024112SP0001-04==
std_msgs/Header header  # timestamp

#==台車操作指令==
uint16 set_flags_1      # 操作指令１

#==台車情報==
uint16 response_flags_1 # 応答１
uint16 inout_flags_1    # 入出力１

#==台車異常==
uint16 error_code       # 異常コード
uint16 error_flags_1    # 異常フラグ１
uint16 error_flags_2    # 異常フラグ２
uint16 error_flags_3    # 異常フラグ３

#==台車オドメトリ==
int32 x                 # 現在地X [0.001m]
int32 y                 # 現在地Y [0.001m]
int16 w                 # 現在姿勢W [0.001rad]

#==台車速度==
int16 vx                # X方向速度 [0.01m/min]
int16 vy                # Y方向速度 [0.01m/min]
int16 vw                # 回転速度 [0.01rad/min]

#==モーター情報==
int32[] motor_pulse_count   # モータードライバのパスルカウント
uint32[] motor_load_ratio   # モーター負荷率
uint32[] motor_current      # モーター電流
#uint32[] motor_torque       # モータートルク
#uint32[] motor_temperature  # モーター温度
#uint32[] driver_temperature # モータードライバの温度