# Control command sent to the STM32 ECU.
# Mirrors message 0x02 in the FS UART protocol.
# NOTE: This is a copy of pedal_node/fs_uart_ws/src/fs_msgs/msg/VehicleControl.msg
# kept here so the simulator container can build and use fs_msgs without needing
# access to the pedal_node submodule. Keep in sync with the pedal_node version.

std_msgs/Header header

# Steering target in degrees. Bridge will clamp to +/- 92 deg.
float32 steering_angle

# Throttle and brake, 0..100. Bridge will clamp.
uint8 throttle
uint8 brake

# Coordination flags forwarded into the protocol's PC flags byte.
bool slam_ready
bool mission_finished
