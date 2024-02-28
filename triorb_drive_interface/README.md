[../](../README.md)

# Package: triorb_drive_interface
## triorb_drive_interface Types

### triorb_drive_interface/msg/MotorParams
```bash
bool lpf                # Use LPF for driving command filter (False: moving average)
uint8 filter_t          # Command filter time constant (0-200)[ms]
uint8 pos_p_gain        # Position loop gain (1-50)[Hz]
uint16 speed_p_gain     # speed loop gain (1-500) [Hz]
uint16 speed_i_gain     # speed loop integral time constant (1-10000) [0.01ms]
uint16 torque_filter    # torque filter (0-4700) [Hz]
uint8 speed_ff          # speed feed-forward (0-100) [%]
uint8 stiffness         # machine stiffness selection (0-15)
```

### triorb_drive_interface/srv/MotorParams
```bash
std_msgs/Empty request
---
MotorParams result
```

### triorb_drive_interface/msg/MotorStatus
```bash
std_msgs/Header header      # Timestamp
uint16 last_error_value     # Last motor alert flag
uint8 last_error_motor      # Motor ID of the last alert
float32 voltage             # Mains voltage observed by the motor driver
uint16 state                # Operating state of each motor (bit flag)
float32 power               # Power consumption of each motor (W)

#---Operating state of each motor (bit flag)---
# 0x8000: Remote control Y button
# 0x4000: Remote control B button
# 0x2000: Remote control A button
# 0x1000: Remote control X button
# 0x0800: Rotating
# 0x0400: Position control complete
# 0x0200: Excitation in progress
# 0x0100: Motor status acquired successfully
```

### triorb_drive_interface/msg/DriveGains
```bash
float32 xy_p    # translation P gain
float32 xy_i    # translation I gain (0 recommended)
float32 xy_d    # translation D gain
float32 w_p     # rotation P gain
float32 w_i     # rotation I gain (0 recommended)
float32 w_d     # rotation D gain
```