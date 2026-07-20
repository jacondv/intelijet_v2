from enum import Enum

class PPSCommand(Enum):
    START_PRESCAN = 1
    START_POSTSCAN = 2
    START_COMPARE = 3
    CANCEL_JOB = 4

    OPEN_HOUSING = 5
    CLOSE_HOUSING = 6
    PAUSE_HOUSING = 7

    # Command sent to PLC start at 101

    PLC_START_PRESCAN = 101
    PLC_START_POSTSCAN = 102
    PLC_OPEN_HOUSING = 103
    PLC_CLOSE_HOUSING = 104
    PLC_PAUSE_HOUSING = 105
    PLC_SET_HOME_POSITION = 106

    #Command Set speed for Housing
    PLC_SET_RETRACT_SPEED = 201
    PLC_SET_EXTEND_SPEED = 202 






