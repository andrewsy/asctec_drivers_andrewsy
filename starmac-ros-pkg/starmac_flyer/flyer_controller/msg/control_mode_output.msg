Header header
string control_mode
bool motors_on
float64 roll_cmd # deg
float64 pitch_cmd # deg
bool direct_yaw_rate_commands # set true to send yaw rate commands, false to send yaw angle
float64 yaw_cmd # deg
float64 yaw_rate_cmd # deg/s
bool direct_thrust_commands # set true to send thrust directly
float64 alt_cmd # m - ignored if direct_thrust_commands = true
float64 thrust_cmd # N - ignored if direct_thrust_commands = false
