#!/usr/bin/python
from numpy import round

# constants of the CAN protocol to control fibre positioner units

#  number of buses on one gateway
BUSES_PER_GATEWAY =  5
# number of FPUs on one CAN bus
FPUS_PER_BUS = 76

MSG_TYPE_DELY = 6

CAN_PROTOCOL_VERSION = 2

MAX_WAVE_ENTRIES = 256
LEN_SERIAL_NUMBER = 6
DEFAULT_SEGMENT_LENGTH_MS = 125
SEGMENT_LENGTH_BASE_UNIT = 100e-9 # = 100 nanoseconds
DEFAULT_TICKS_PER_SEGMENT = round(DEFAULT_SEGMENT_LENGTH_MS * 1e-3 / SEGMENT_LENGTH_BASE_UNIT)

# FPU states

FPST_UNKNOWN                 = 0
FPST_UNINITIALIZED           = 1
FPST_LOCKED                  = 2
FPST_DATUM_SEARCH            = 3
FPST_AT_DATUM                = 4
FPST_LOADING                 = 5
FPST_READY_FORWARD           = 6
FPST_READY_REVERSE           = 7
FPST_MOVING                  = 8
FPST_RESTING                 = 9
FPST_ABORTED                 = 10
FPST_OBSTACLE_ERROR          = 11

NUM_FPU_STATES               = 12


# command codes

CCMD_NO_COMMAND                       = 0  # reserved
CCMD_CONFIG_MOTION                    = 1  # configure waveform
CCMD_EXECUTE_MOTION                   = 2  # execute loaded waveform
CCMD_ABORT_MOTION                     = 3  # abort any ongoing movement


CCMD_READ_REGISTER                    = 6  # read register
CCMD_PING_FPU                         = 7  # check connectivity
CCMD_RESET_FPU                        = 8  # reset MCU
CCMD_FIND_DATUM                       = 9  # "automatic" datum search
CCMD_RESET_STEPCOUNTER                = 10 # only for debugging
CCMD_REPEAT_MOTION                    = 11 # re-use last waveform
CCMD_REVERSE_MOTION                   = 12 # invert last waveform
CCMD_ENABLE_BETA_COLLISION_PROTECTION = 13 # "ENABLE_COLLIDE"
CCMD_FREE_BETA_COLLISION              = 14 # "FREE_COLLIDE"
CCMD_SET_USTEP_LEVEL                  = 15 # set motor micro-stepping (1,2,4,8 supported)

CCMD_LOCK_UNIT                        = 4 # ignore any command except reset and unlock
CCMD_UNLOCK_UNIT                      = 5 # listen to commands again

CCMD_GET_FIRMWARE_VERSION             = 16 # get firmware version
CCMD_CHECK_INTEGRITY                  = 17 # report firmware CRC
CCMD_FREE_ALPHA_LIMIT_BREACH          = 18 # untangle alpha arm
CCMD_ENABLE_ALPHA_LIMIT_PROTECTION    = 19 # re-enable limit switch
CCMD_SET_TICKS_PER_SEGMENT            = 20 # set movement time interval
CCMD_SET_STEPS_PER_SEGMENT            = 21 # set minimum step frequency
CCMD_ENABLE_MOVE                      = 22 # set minimum step frequency
CCMD_READ_SERIAL_NUMBER               = 23 # read serial number from NVRAM
CCMD_WRITE_SERIAL_NUMBER              = 24 # write serial number to NVRAM

CMSG_FINISHED_MOTION                  = 25 # executeMotion finished
CMSG_FINISHED_DATUM                   = 26 # findDatum finished
CMSG_WARN_COLLISION_BETA              = 27 # collision at beta arm
CMSG_WARN_LIMIT_ALPHA                 = 28 # limit switch at alpha arm
CMSG_WARN_TIMEOUT_DATUM               = 29 # datum search time out
CMSG_WARN_CANOVERFLOW                 = 30 # CAN buffer overflow warning

NUM_CAN_COMMANDS = 30


# error codes
MCE_FPU_OK			     = 0x00	##  no error
MCE_WARN_COLLISION_DETECTED	     = 0x01	##  beta collision warning
MCE_WARN_LIMIT_SWITCH_BREACH     = 0x02	##  alpha limit switch breach
MCE_ERR_INVALID_COMMAND	     = 0x03	##  invalid command received by motion controller
MCE_NOTIFY_COMMAND_IGNORED	     = 0x04	##  command was ignored by FPU motion controller
MCE_ERR_WAVEFORM_NOT_READY	     = 0x05	##  waveform not ready for execution
MCE_WAVEFORM_REJECTED	     = 0x06	##  too many waveform entries
MCE_WARN_STEP_TIMING_ERROR	     = 0x07	##  Microstepping value is too high for step frequency
MCE_ERR_INVALID_PARAMETER	     = 0x08	##  invalid parameter was rejected by motion controller
MCE_ERR_DATUM_TIME_OUT	     = 0x09	##  datum search exceeded hardware time or step limit
MCE_NOTIFY_DATUM_ALPHA_ONLY	     = 0x0a	##  only the alpha arm was moved to datum
MCE_NOTIFY_DATUM_BETA_ONLY	     = 0x0b	##  only the beta arm was moved to datum
MCE_ERR_AUTO_DATUM_UNINITIALIZED = 0x0c	##  automatic datum operation was requested, but FPU is not initialized
MCE_ERR_DATUM_ON_LIMIT_SWITCH    = 0x0d	##  datum command was rejected because alpha arm is on limit switch
MCE_ERR_CAN_OVERFLOW_HW	     = 0x0e	##  overflow in CAN hardware buffer
MCE_ERR_CAN_OVERFLOW_SW	     = 0x0f	##  CAN overflow in motion controller firmware buffer


WAVEFORM_OK                  = 0x00
WAVEFORM_TOO_BIG	     = 0x01	##  too many waveform entries
WAVEFORM_SEQUENCE	     = 0x02	##  transmitted waveform sequence not consistent in respect to use of first and last flags
WAVEFORM_BADVALUE	     = 0x03	##  the transmitted waveform value did not pass bounds checking
WAVEFORM_UNDEFINED           = 0x04     ## no waveform data defined because of other error

STBT_ALPHA_DATUM_ACTIVE   = 1          ## alpha datum switch is active
STBT_BETA_DATUM_ACTIVE    = 1 << 1     ## beta datum switch is active
STBT_COLLISION_DETECTED   = 1 << 2     ## collision was detected
STBT_ALPHA_AT_LIMIT	      = 1 << 3     ## alpha limit switch active
STBT_FPU_LOCKED	      = 1 << 4     ## FPU is currently locked
STBT_ALPHA_LAST_DIRECTION = 1 << 5     ## last movement direction of alpha arm
STBT_BETA_LAST_DIRECTION  = 1 << 6     ## last movement direction of beta arm
STBT_IS_ZEROED	      = 1 << 7     ## both arms have been datumed
STBT_WAVEFORM_VALID	      = 1 << 8     ## loaded waveform is valid
STBT_WAVEFORM_READY	      = 1 << 9     ## ready to run waveform
STBT_WAVEFORM_REVERSED    = 1 << 10    ## waveform is in reversed mode


# datum option flags

DATUM_SKIP_ALPHA = 1
DATUM_SKIP_BETA = (1 << 1)
DATUM_MODE_AUTO = (1 << 2)
DATUM_MODE_ANTI_CLOCKWISE = (1 << 3)
DATUM_TIMEOUT_DISABLE = (1 << 4)

# sign bit of current or last movement
DIRST_ANTI_CLOCKWISE = 0
DIRST_CLOCKWISE = 1

# default priority for CAN messages from FPU
# to driver
DEFAULT_PRIORITY = 0b0010

priorities_by_cmd_id = {
    CMSG_WARN_COLLISION_BETA                 : 0b0001,
    CMSG_WARN_LIMIT_ALPHA                    : 0b0001,
    CMSG_WARN_TIMEOUT_DATUM                  : 0b0001,
    CMSG_WARN_CANOVERFLOW                    : 0b0001, }
