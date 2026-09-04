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

#==action executorがActionExecutionReport/ActionExecutionCommandで利用する定数をまとめる定数専用msg==
# 利用側はActionExecutionReportDetail.PHASE_*等を参照し、primitive fieldへ格納する利用方法を想定

# phase
uint8 PHASE_UNKNOWN=0
uint8 PHASE_STARTING=1
uint8 PHASE_RUNNING=2
uint8 PHASE_PAUSED=3
uint8 PHASE_RESUME=4
uint8 PHASE_CANCELLING=5
uint8 PHASE_SUCCEEDED=6
uint8 PHASE_FAILED=7
uint8 PHASE_REJECTED=8

# result
uint8 RESULT_SUCCEEDED=9
uint8 RESULT_REJECTED=10
uint8 RESULT_FAILED=11
uint8 RESULT_UNKNOWN=12

# reason_code
uint32 REASON_NONE=0
uint32 REASON_FORCE_SUCCESS=1
uint32 REASON_BUSY=2
uint32 REASON_RESULT_TIMEOUT=3
uint32 REASON_RESULT_REJECTED=4
uint32 REASON_PAUSE_UNSUPPORTED=5
uint32 REASON_COMMAND_NOT_ALLOWED=6                # command service受付専用
uint32 REASON_CONTROL_IN_PROGRESS=7                # command service受付専用
uint32 REASON_INVALID_ACTION_CONTROL_REQUEST_ID=8  # command service受付専用
uint32 REASON_NAVIGATION_TRANSFORM_FAILE=9         # automove 詳細情報引継ぎ
uint32 REASON_NAVIGATION_NO_CHANGE_TIMESTAMP=10    # automove 詳細情報引継ぎ
uint32 REASON_NAVIGATION_POSE_NOT_RECEIVED=11      # automove 詳細情報引継ぎ
uint32 REASON_NAVIGATION_POSE_UNUSABLE=12          # automove 詳細情報引継ぎ
uint32 REASON_NAVIGATION_STEP_TIMEOUT=13           # automove 詳細情報引継ぎ
