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

#==自律移動のゲインパラメーター==
float32 xy_p    # translation P gain
float32 xy_i    # translation I gain (0 recommended)
float32 xy_d    # translation D gain
float32 w_p     # rotation P gain
float32 w_i     # rotation I gain (0 recommended)
float32 w_d     # rotation D gain