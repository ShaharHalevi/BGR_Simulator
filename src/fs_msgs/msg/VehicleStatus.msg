# Vehicle status as reported by the STM32 ECU.
# Mirrors message 0x10 in the FS UART protocol.
# NOTE: This is a copy of pedal_node/fs_uart_ws/src/fs_msgs/msg/VehicleStatus.msg
# kept here so the simulator container can build and use fs_msgs without needing
# access to the pedal_node submodule. Keep in sync with the pedal_node version.

std_msgs/Header header

# Autonomous System state machine
uint8 AS_STATE_OFF       = 0
uint8 AS_STATE_READY     = 1
uint8 AS_STATE_DRIVING   = 2
uint8 AS_STATE_FINISHED  = 3
uint8 AS_STATE_EMERGENCY = 4

uint8 as_state
uint8 mission_id

# Status flags (decoded from the protocol's status byte)
bool ebs_armed
bool sensor_error
