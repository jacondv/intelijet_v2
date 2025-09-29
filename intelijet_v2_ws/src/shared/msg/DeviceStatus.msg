string name
string device_state
string process_state
string mode
string detail
time   last_update

# ---- Constants for device_state ----
string CONNECTED    = Connected
string DISCONNECTED = Disconnected
string ERROR        = Error
string PRESCAN_ERROR = PrescanningError
string POSTSCAN_ERROR = PostscanningError

# ---- Constants for process_state ----
string IDLE         = Idle
string PROCESSING   = Processing
string FINISHED     = Finished
string STANDBY      = Standby
string PRESCAN     = Prescanning
string POSTSCAN    = Postscanning
string OPEN_HOUSING    = HousingOpening
string CLOSE_HOUSING   = HousingClosing
# ---- Constants for mode ----
string CONTINUOUS   = Continuous
string ON_DEMAND    = On-demand