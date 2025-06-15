# Standard Header for timestamp and frame id
std_msgs/Header header

# Robot's battery voltage (Volts)
float32 battery_voltage

# Current robot mode (0: Idle, 1: Moving, 2: Error, 3: Charging)
int8 mode

# A short status description
string status_message

# Current position (x, y, z)
float32[3] position

# List of active error codes
int32[] error_codes

# Is the robot currently performing an action?
bool is_acting