# Command published by automove_task to control path_planner.
uint8 COMMAND_SET_POS_TARGET=0
uint8 COMMAND_PAUSE=1
uint8 COMMAND_TERMINATE=2
uint8 COMMAND_RESUME=3

uint8 command

# Used only when command is COMMAND_SET_POS_TARGET.
triorb_drive_interface/TriorbSetPos3 target

# Termination reason.
# Used only when command is COMMAND_TERMINATE.

uint16 TIMEOUT_FAILED=0
uint16 FORCE_STOP=4
uint16 BLOCKED_BY_PERMIT=10

uint16 terminate_reason