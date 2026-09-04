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

#==task orchestratorに対するアクション実行の状況・結果報告==
ActionExecutionKey key       # action_instance_idとexecution_idの共通相関Key
uint16 action_type_id        # デバッグ時の可読性用
uint8 phase                  # ActionExecutionReportDetail.msgのPHASE_*で定義された値
uint32 reason_code           # ActionExecutionReportDetail.msgのREASON_*で定義された値
uint8 result                 # ActionExecutionReportDetail.msgのRESULT_*で定義された値
string detail                # debug向け文字列
