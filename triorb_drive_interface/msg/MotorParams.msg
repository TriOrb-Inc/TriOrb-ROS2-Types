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

#==モーター制御パラメーター==
bool lpf                # Use LPF for driving command filter (False: moving average)
uint8 filter_t          # Command filter time constant (0-200)[ms]
uint8 pos_p_gain        # Position loop gain (1-50)[Hz]
uint16 speed_p_gain     # speed loop gain (1-500) [Hz]
uint16 speed_i_gain     # speed loop integral time constant (1-10000) [0.01ms]
uint16 torque_filter    # torque filter (0-4700) [Hz]
uint8 speed_ff          # speed feed-forward (0-100) [%]
uint8 stiffness         # machine stiffness selection (0-15)
