string name
uint8 device_state
uint8 process_state
uint8 mode
string detail
time   last_update

# ---- Constants for device_state ----
string CONNECTED    = Connected
string DISCONNECTED = Disconnected
string ERROR        = Error

# ---- Constants for process_state ----
string IDLE         = Idle
string PROCESSING   = Processing
string FINISHED     = Finished
string STANDBY      = Standby

# ---- Constants for mode ----
string CONTINUOUS   = Continuous
string ON_DEMAND    = On-demand