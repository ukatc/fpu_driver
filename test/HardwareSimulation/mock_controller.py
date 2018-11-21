#!/usr/bin/python

"""
This is a mock FPU grid controller which decodes and
synthesizes a response.

"""

from __future__ import print_function
import os

import codec
from fpu_sim import FPU, FPUGrid

from gevent import sleep, spawn, spawn_later
from fpu_sim import LEN_SERIAL_NUMBER

#  number of buses on one gateway
BUSES_PER_GATEWAY =  5
# number of FPUs on one CAN bus
FPUS_PER_BUS = 76

MSG_TYPE_DELY = 6

CAN_PROTOCOL_VERSION = 1


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

if CAN_PROTOCOL_VERSION == 1:
    # In version 2, the following two are covered by the ping command, which also
    # reports the current positions.
    CCMD_GET_STEPS_ALPHA                  = 4 # get alpha counts
    CCMD_GET_STEPS_BETA                   = 5 # get beta counts
    # in version 2, these are covered by the findDatum response
    CCMD_GET_ERROR_ALPHA                  = 16 # get residue count at last datum hit
    CCMD_GET_ERROR_BETA                   = 17 # get residue count at last datum hit

    CCMD_READ_SERIAL_NUMBER               = 18 # read serial number from NVRAM
    CCMD_WRITE_SERIAL_NUMBER              = 19 # write serial number to NVRAM
    NUM_CAN_COMMANDS = 20
    
    # code 101 unused 
    # code 102 unused 
    CMSG_FINISHED_MOTION               = 103 #  executeMotion finished
    CMSG_FINISHED_DATUM                = 104 #  findDatum finished
    CMSG_WARN_COLLISION_BETA           = 105 #  collision at beta arm
    CMSG_WARN_LIMIT_ALPHA              = 106 #  limit switch at alpha arm
    CMSG_WARN_RACE                     = 107 #  step timing error
    CMSG_WARN_CANOVERFLOW              = 108 #  CAN overflow
else:
    CCMD_LOCK_UNIT                        = 4 # ignore any command except reset and unlock
    CCMD_UNLOCK_UNIT                      = 5 # listen to commands again

    # commands which are not yet implemented
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
if CAN_PROTOCOL_VERSION == 1:
    ER_STALLX           = 0x01        #  x motor stall (obsolete)
    ER_STALLY           = 0x02        #  y motor stall (obsolete)
    ER_COLLIDE          = 0x03        #  FPU collision detected
    ER_INVALID          = 0x04        #  received command not valid
    ER_WAVENRDY         = 0x05        #  waveform not ready 
    ER_WAVE2BIG         = 0x06        #  waveform exceeds memory allocation
    ER_TIMING           = 0x07        #  step timing error (interrupt race condition)
    ER_M1LIMIT          = 0x08        #  M1 Limit switch breached
    ER_M2LIMIT          = 0x09        #  no longer used
    ER_CANOVRS          = 0x0A        #  CAN overflow firmware software buffer
    ER_CANOVRH          = 0x0B        #  CAN overflow FPU hardware buffer
    ER_PARAM            = 0x10        #  parameter out of range
    ER_AUTO             = 0x11        #  FPU cannot datum automatically
    ER_DATUMTO          = 0x12        #  hardware error: datum search timed out by firmware
    ER_DATUM_LIMIT      = 0x13        #  datum search denied, limit switch is active
    # (note: the driver uses additional internal status codes)
else:
    MCE_FPU_OK			     = 0x00	##  no error
    MCE_WARN_COLLISION_DETECTED	     = 0x01	##  beta collision warning
    MCE_WARN_LIMIT_SWITCH_BREACH     = 0x02	##  alpha limit switch breach
    MCE_ERR_INVALID_COMMAND	     = 0x03	##  invalid command received by motion controller
    MCE_NOTIFY_COMMAND_IGNORED	     = 0x04	##  command was ignored by FPU motion controller
    MCE_ERR_WAVEFORM_NOT_READY	     = 0x05	##  waveform not ready for execution
    MCE_WAVEFORM_TOO_BIG	     = 0x06	##  too many waveform entries
    MCE_WAVEFORM_SEQUENCE	     = 0x07	##  transmitted waveform sequence not consistent in respect to use of first and last flags
    MCE_WAVEFORM_BADVALUE	     = 0x08	##  the transmitted waveform value did not pass bounds checking
    MCE_WARN_STEP_TIMING_ERROR	     = 0x09	##  Microstepping value is too high for step frequency
    MCE_ERR_INVALID_PARAMETER	     = 0x0a	##  invalid parameter was rejected by motion controller
    MCE_ERR_DATUM_TIME_OUT	     = 0x0b	##  datum search exceeded hardware time or step limit
    MCE_NOTIFY_DATUM_ALPHA_ONLY	     = 0x0c	##  only the alpha arm was moved to datum
    MCE_NOTIFY_DATUM_BETA_ONLY	     = 0x0d	##  only the beta arm was moved to datum
    MCE_ERR_AUTO_DATUM_UNINITIALIZED = 0x0e	##  automatic datum operation was requested, but FPU is not initialized
    MCE_ERR_DATUM_ON_LIMIT_SWITCH    = 0x0f	##  datum command was rejected because alpha arm is on limit switch
    MCE_ERR_CAN_OVERFLOW_HW	     = 0x10	##  overflow in CAN hardware buffer
    MCE_ERR_CAN_OVERFLOW_SW	     = 0x11	##  CAN overflow in motion controller firmware buffer
    MCE_NO_CONFIRMATION_EXPECTED     = 0x12     ##  command was sent for which no confirmation is expected
    MCE_COMMAND_TIMEDOUT             = 0x13     ##  response for CAN command is missing (timed out)



if CAN_PROTOCOL_VERSION == 1:
    # status flags
    STBT_MSGRCV          = 0        #  message received over CANBUS
    STBT_WAVE_READY      = 1 << 1   #  waveform good and ready for execution
    STBT_EXECUTE_WAVE    = 1 << 2   #  internal start flag to start executing waveform
    STBT_RUNNING_WAVE    = 1 << 3   #  FPU is running the waveform
    STBT_ABORT_WAVE      = 1 << 4   #  abort waveform
    STBT_M1LIMIT         = 1 << 5   #  M1 Limit breached 
    STBT_M2LIMIT         = 1 << 6   #  no longer used
    STBT_REVERSE_WAVE    = 1 << 7   #  waveform to be run in reverse
else:
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

def encode_and_send(msg, socket, verbose=False):
    confirmation = codec.encode(msg, verbose=verbose)
    socket.sendall(confirmation)


def fold_stepcount_alpha(val):
    low_limit = - 10000
    vrange = 1 << 16
    # As specified in protcol,
    # cap underflow / overflow values to representable range
    if val < low_limit:
        val = low_limit
    if val > low_limit + vrange:
        val = low_limit + vrange

    # convert to unsigned 16-bit number
            
    if val < 0:
        val += (1 << 16)

    return val

def fold_stepcount_beta(val):
    low_limit = - 0x8000
    vrange = 1 << 16
    # cap underflow / overflow values to representable range
    if val < low_limit:
        val = low_limit
    if val > low_limit + vrange:
        val = low_limit + vrange
    # convert to unsigned 16-bit number
            
    if val < 0:
        val += (1 << 16)

    return val


def fold_stepcount_deviation(val):
    low_limit = - 0x8000
    vrange = 1 << 16
    assert( (val >= low_limit) and (val < (low_limit + vrange)))
    # convert to unsigned 16-bit number
            
    if val < 0:
        val += (1 << 16)

    return val


def getStatus(FPU):
    status = 0
    if FPU.alpha_datum_active:
        status |=  STBT_ALPHA_DATUM_ACTIVE
        
    if FPU.beta_datum_active:
        status |=  STBT_BETA_DATUM_ACTIVE
        
    if FPU.is_collided:
        status |=  STBT_COLLISION_DETECTED
        
    if FPU.alpha_limit_breach:
        status |= STBT_ALPHA_AT_LIMIT 
        
    if FPU.fpu_locked:
        status |= STBT_FPU_LOCKED
        
    # the following two fields are for recovery porposes - they
    # don't have a meaningful value before the first movement
    # has started (we could use a third state, but this isn't worth the
    # extra bits which are expensive)
    if FPU.alpha_last_direction == DIRST_CLOCKWISE:
        status |=  ALPHA_LAST_DIRECTION
        
    if FPU.beta_last_direction == DIRST_CLOCKWISE:
        status |=  beta_LAST_DIRECTION

    # alpha and beta state are lumped together here -
    # the common case, extra info goes in findDatum status code
    if FPU.was_initialized:
        status |=  IS_ZEROED
        
    if FPU.wave_ready:
        status |=  STBT_WAVEFORM_READY
        
    if FPU.wave_reversed:
        status |= STBT_WAVEFORM_REVERSED
                
    return (status, fpu.state )


def create_gwheader(fpu_adr_bus, bus_adr, prio=0x02):
    # CAN header for gateway
    tx_prio = prio
    tx_canid = (tx_prio << 7) | fpu_adr_bus
    
    TH = [ 0 ] * 3
    TH[0] = bus_adr
    TH[1] = (tx_canid & 0xff)
    TH[2] = ((tx_canid >> 8) & 0xff)



HDR_SEQNUM = 1
HDR_COMMAND_ID = 2
HDR_STWORD = 4
HDR_FPUSTATE = 8
HDR_ECODE = 16
HDR_STEPCOUNTS = 32

DEFAULT_HEADER = (HDR_SEQNUM | HDR_COMMAND_ID | HDR_STWORD 
                  | HDR_FPUSTATE | HDR_ECODE | HDR_STEPCOUNTS )


def create_CANheader(command_id, fpu_id, seqnum, ecode=MCE_FPU_OK, fields=DEFAULT_HEADER):
    TX = [ 0 ] * 8

    status_word, fpustate = getStatus(FPUGrid[fpu_id])
    if fields & HDR_SEQNUM:
        TX[0] = seqnum
        
    if fields & HDR_COMMANDID:
        TX[1] = command_id & 0x1f
        
    if fields & STWORD:
        TX[1] = TX[1] |  ( ( 0x7 & status_word) << 5)
        TX[2] = 0xff & ( status_word > 3)
        
    if fields & HDR_FPUSTATE:
        TX[3] = (fpu_state & 0x0f)
        
    if fields & HDR_ECODE:
        TX[3] = TX[3] | (0xf0 & (ecode << 4))


    if fields & HDR_STEPCOUNTS:
        pos_alpha = fold_stepcount_alpha(FPUGrid[fpu_id].alpha_steps)
        TX[4] = count0 = pos_alpha & 0xff
        TX[5] = count1 = (pos_alpha >> 8) & 0xff
    
        pos_beta = fold_stepcount_beta(FPUGrid[fpu_id].beta_steps)
        TX[6] = count2 = pos_beta & 0xff
        TX[7] = count3 = (pos_beta >> 8) & 0xff
        
        
def handle_configMotion(fpu_id, fpu_adr_bus, bus_adr, RX, verbose=0):

    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpu_adr_bus
    


    command_id = RX[0]
    tx3_errflag = 0

    first_entry = RX[1] & 1
    if first_entry and verbose:
        print("first_entry set!")
    last_entry = (RX[1] >> 1) & 1
    if last_entry and verbose:
        print("last_entry set!")

    # work around a bug in firmware version 1
    BUG_WORKAROUND = 1
    if BUG_WORKAROUND:
        astep = ((RX[2] &  0x3f) << 8) + RX[3]
        apause = (RX[2] >> 6) & 1
        if apause:
            astep = 0
        aclockwise = (RX[2] >> 7) & 1

        bstep = ((RX[4] &  0x3f) << 8) + RX[5]
        bpause = (RX[4] >> 6) & 1
        if bpause:
            bstep=0
        bclockwise = (RX[4] >> 7) & 1
    else:
        astep = ((RX[3] &  0x3f) << 8) + RX[2]
        apause = (RX[3] >> 6) & 1
        if apause:
            astep = 0
        aclockwise = (RX[3] >> 7) & 1

        bstep = ((RX[5] &  0x3f) << 8) + RX[4]
        bpause = (RX[5] >> 6) & 1
        if bpause:
            bstep=0
        bclockwise = (RX[5] >> 7) & 1

        
    nwave_entries = 0
    if verbose:
        print("FPU #%i command =%i , rx=%s" % (fpu_id, command_id, RX))
    
    try:
        FPUGrid[fpu_id].addStep(first_entry, last_entry,
                                astep, apause, aclockwise,
                                bstep, bpause, bclockwise)

        tx3_errflag = 0
        tx4_errcode = 0
        nwave_entries = FPUGrid[fpu_id].nwave_entries
    
    except IndexError:
        tx3_errflag = 0xff
        tx4_errcode = ER_PARAM
    except RuntimeError:
        tx3_errflag = 0xff
        tx4_errcode = ER_INVALID
    except BufferError:
        tx3_errflag = 0xff
        tx4_errcode = ER_CANOVRH
        command_id = CMSG_WARN_CANOVERFLOW
    except OverflowError:
        tx3_errflag = 0xff
        tx4_errcode = ER_CANOVRS
        command_id = CMSG_WARN_CANOVERFLOW
        
        
        


    if CAN_PROTOCOL_VERSION == 1:
        send_confirmation = True
    else:
        send_confirmation = first_entry or last_entry or (tx3_errflag != 0)
        
    if send_confirmation:
        TH = [ 0 ] * 3
        TH[0] = bus_adr
        TH[1] = (tx_canid & 0xff)
        TH[2] = ((tx_canid >> 8) & 0xff)
        
        TX = [0] * 8
        TX[0] = fpu_adr_bus
        TX[1] = command_id
        TX[2] = getStatus(FPUGrid[fpu_id]) 
        TX[3] = tx3_errflag        
        TX[4] = tx4_errcode if tx3_errflag != 0 else nwave_entries
        TX[5] = dummy1 = 0
        TX[6] = dummy2 = 0
        TX[7] = dummy3 = 0
        
        confirmation = TH + TX 
        return confirmation
    else:
        # no confirmation
        return None



def handle_pingFPU(fpu_id, fpu_adr_bus, bus_adr, RX):
    seqnum = RX[0]
    command_id = RX[1] & 0x1f
    
    TH = create_gwheader(fpu_adr_bus, bus_adr)
    TX = create_CANheader(command_id, fpu_id, seqnum, ecode=MCE_FPU_OK)
    
    return TH + TX


def handle_abortMotion(fpu_id, fpu_adr_bus, bus_adr, RX):
    command_id = RX[0]

    # CAN header for gateway
    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpu_adr_bus
    
    TH = [ 0 ] * 3
    TH[0] = bus_adr
    TH[1] = (tx_canid & 0xff)
    TH[2] = ((tx_canid >> 8) & 0xff)


    FPUGrid[fpu_id].abortMotion(fpu_id)
    
    TX = [ 0 ] * 8
    
    TX[0] = fpu_adr_bus
    TX[1] = command_id
    TX[2] = getStatus(FPUGrid[fpu_id])
    TX[3] = errflag = 0


    
    return TH + TX


def handle_freeBetaCollision(fpu_id, fpu_adr_bus, bus_adr, RX):
    command_id = RX[0]

    # CAN header for gateway
    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpu_adr_bus
    
    TH = [ 0 ] * 3
    TH[0] = bus_adr
    TH[1] = (tx_canid & 0xff)
    TH[2] = ((tx_canid >> 8) & 0xff)

    direction = RX[1]
    assert( (direction == 0) or (direction == 1))
    FPUGrid[fpu_id].freeBetaCollision(direction)
    
    TX = [ 0 ] * 8
    
    TX[0] = fpu_adr_bus
    TX[1] = command_id
    TX[2] = getStatus(FPUGrid[fpu_id])
    TX[3] = errflag = 0


    
    return TH + TX


def handle_enableBetaCollisionProtection(fpu_id, fpu_adr_bus, bus_adr, RX):
    command_id = RX[0]

    # CAN header for gateway
    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpu_adr_bus
    
    TH = [ 0 ] * 3
    TH[0] = bus_adr
    TH[1] = (tx_canid & 0xff)
    TH[2] = ((tx_canid >> 8) & 0xff)
    
    FPUGrid[fpu_id].enableBetaCollisionProtection()
    
    TX = [ 0 ] * 8
    
    TX[0] = fpu_adr_bus
    TX[1] = command_id
    TX[2] = getStatus(FPUGrid[fpu_id])
    TX[3] = errflag = 0


    
    return TH + TX


def handle_setUStepLevel(fpu_id, fpu_adr_bus, bus_adr, RX):
    command_id = RX[0]

    # CAN header for gateway
    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpu_adr_bus
    
    TH = [ 0 ] * 3
    TH[0] = bus_adr
    TH[1] = (tx_canid & 0xff)
    TH[2] = ((tx_canid >> 8) & 0xff)

    ustep_level = RX[1]
    if ustep_level in [1,2,4,8]:
        FPUGrid[fpu_id].setUStepLevel(ustep_level)
        errflag = 0
        ecode = 0
    else:
        erflag = 0xff
        ecode = ER_PARAM
    
    TX = [ 0 ] * 8
    
    TX[0] = fpu_adr_bus
    TX[1] = command_id
    TX[2] = getStatus(FPUGrid[fpu_id])
    TX[3] = errflag
    TX[4] = ecode


    
    return TH + TX


def handle_readRegister(fpu_id, fpu_adr_bus, bus_adr, RX):

    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpu_adr_bus
    
    TH = [ 0 ] * 3
    TH[0] = bus_adr
    TH[1] = (tx_canid & 0xff)
    TH[2] = ((tx_canid >> 8) & 0xff)

    TX = [0] * 8

    command_id = RX[0]
    register_address = ((RX[1] << 8) | RX[2])

    TX[0] = fpu_adr_bus
    TX[1] = command_id
    TX[2] = getStatus(FPUGrid[fpu_id]) 
    TX[3] = errflag = 0

    byte = FPUGrid[fpu_id].getRegister(register_address)
    TX[4] = byte
    
    print("fpu #%i: read from address 0x%04x yields 0x%02x" %
          (fpu_id,register_address, byte) )
    
    return TH + TX 


def handle_resetFPU(fpu_id, fpu_adr_bus, bus_adr, RX, socket, verbose=False):

    def reset_func(fpu_id, fpu_adr_bus, bus_adr, RX, socket, verbose=False):

        FPUGrid[fpu_id].resetFPU(fpu_id, sleep);

        # response message packet        
        seqnum = RX[0]
        command_id = RX[1] & 0x1f
    
        TH = create_gwheader(fpu_adr_bus, bus_adr)
        TX = create_CANheader(command_id, fpu_id, seqnum, ecode=MCE_FPU_OK)
        TX[4] = count0 = 0
        TX[5] = count1 = 0    
        TX[6] = count2 = 0
        TX[7] = count3 = 0

        conf_msg = TH + TX 

        print("fpu #%i: sending reset confirmation" % fpu_id)
        encode_and_send(conf_msg, socket, verbose=verbose)

    spawn(reset_func, fpu_id, fpu_adr_bus, bus_adr, RX, socket, verbose=verbose)
        
    print("returning from reset message")
    
    return None

def handle_invalidCommand(fpu_id, fpu_adr_bus, bus_adr, RX):

    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpu_adr_bus

    ## gateway header
    TH = [ 0 ] * 3
    
    TH[0] = bus_adr
    TH[1] = (tx_canid & 0xff)
    TH[2] = ((tx_canid >> 8) & 0xff)
    

    command_id = RX[0]
    
    # response message packet
    TX = [0] * 8
    TX[0] = fpu_adr_bus
    TX[1] = command_id
    TX[2] = getStatus(FPUGrid[fpu_id])
    TX[3] = errflag = 0xff
    TX[4] = errcode = ER_INVALID
    TX[5] = dummy1 = 0    
    TX[6] = dummy2 = 0
    TX[7] = dummy3 = 0
    
    return TH + TX 



##############################
# the following two callback objects are passed as bound methods
# to the FPUs executeMotion and findDatum methods so that FPUs can signal
# collisions or limit switch breaks

class LimitCallback:
    def __init__(self, fpu_adr_bus, bus_adr, socket, verbose=False):
        self.socket = socket
        self.fpu_adr_bus = fpu_adr_bus
        self.bus_adr = bus_adr
        self.verbose=verbose
        
    def call(self, fpu):

        fpu_id = fpu.fpu_id

        tx_prio = 0x01
        tx_canid = (tx_prio << 7) | self.fpu_adr_bus
        
        TH = [ 0 ] * 3
        TH[0] = self.bus_adr
        TH[1] = (tx_canid & 0xff)
        TH[2] = ((tx_canid >> 8) & 0xff)
        
        TX = [0] * 8
        TX[0] = self.fpu_adr_bus
        TX[1] = CMSG_WARN_LIMIT_ALPHA
        TX[2] = status = getStatus(fpu)
        

        TX[3] = errflag = 0xff
        #TX[4] = errcode = ER_COLLIDE
        TX[4] = errcode = ER_M1LIMIT
            
        TX[5] = dummy1 = 0
        TX[6] = dummy2 = 0
        TX[7] = dummy3 = 0

        if self.verbose:
            print("FPU %i: sending limit switch break message" % fpu_id)
        limit_message =  TH + TX
        encode_and_send(limit_message, self.socket, verbose=self.verbose)

class CollisionCallback:
    def __init__(self, fpu_adr_bus, bus_adr, socket, verbose=False):
        self.socket = socket
        self.fpu_adr_bus = fpu_adr_bus
        self.bus_adr = bus_adr
        self.verbose = verbose
        
    def call(self, fpu):

        fpu_id = fpu.fpu_id

        tx_prio = 0x01
        tx_canid = (tx_prio << 7) | self.fpu_adr_bus
        
        TH = [ 0 ] * 3
        TH[0] = self.bus_adr
        TH[1] = (tx_canid & 0xff)
        TH[2] = ((tx_canid >> 8) & 0xff)
        
        TX = [0] * 8
        TX[0] = self.fpu_adr_bus
        TX[1] = CMSG_WARN_COLLISION_BETA
        TX[2] = status = getStatus(fpu)
        

        TX[3] = errflag = 0xff
        TX[4] = errcode = ER_COLLIDE
            
        TX[5] = dummy1 = 0
        TX[6] = dummy2 = 0
        TX[7] = dummy3 = 0

        if self.verbose:
            print("FPU %i: sending collision detection message" % fpu_id)
        
        limit_message =  TH + TX
        encode_and_send(limit_message, self.socket, verbose=self.verbose)


#################################

def handle_findDatum(fpu_id, fpu_adr_bus, bus_adr, RX, socket, opts):
        
    print("starting findDatum for FPU %i" % fpu_id)

    if len(RX) < 8:
        print("CAN command format error, length must be 8");
        return []

    ## gateway header
    
    seqnum = RX[0]
    command_id = RX[1] & 0x1f
    
    TH = create_gwheader(fpu_adr_bus, bus_adr)
    TX = create_CANheader(command_id, fpu_id, seqnum, FPST_OK)
    
    TX[5] = dummy0 = 0
    TX[6] = dummy1 = 0
    TX[7] = dummy2 = 0

    flag_skip_alpha = False
    flag_skip_beta = False
    flag_auto_datum = False
    flag_anti_clockwise = False
    flag_disable_timeout = False

    skip_flag = RX[2]
    flag_skip_alpha = (skip_flag & DATUM_SKIP_ALPHA) > 0
    flag_skip_beta = (skip_flag & DATUM_SKIP_BETA) > 0
            
    flag_auto_datum = (skip_flag & DATUM_MODE_AUTO) > 0
    flag_anti_clockwise = (skip_flag & DATUM_MODE_ANTI_CLOCKWISE) > 0
    flag_disable_timeout = (skip_flag & DATUM_TIMEOUT_DISABLE) > 0


    errcode = FPUGrid[fpu_id].start_findDatum()

    if errcode == MCE_FPU_OK:

        # create closure which sends return message when operation is finished
        # (non-local variables are looked up in the enclosing scope)
        
        def findDatum_func(fpu_id, fpu_adr_bus, bus_adr, RX, socket, opts):

            # instantiate two objects which can send collision messages
            # if needed
            limit_callback = LimitCallback(fpu_adr_bus, bus_adr, socket)
            collision_callback = CollisionCallback(fpu_adr_bus, bus_adr, socket)


            # simulate findDatum FPU operation
            FPUGrid[fpu_id].findDatum(sleep,
                                      limit_callback.call, collision_callback.call,
                                      skip_alpha=flag_skip_alpha, skip_beta=flag_skip_beta,
                                      auto_datum=flag_auto_datum,
                                      anti_clockwise=flag_anti_clockwise,
                                      disable_timeout=flag_disable_timeout)
            
            print("FPU %i: findDatum command finished" % fpu_id);

    
            TH = create_gwheader(fpu_adr_bus, bus_adr)

            status_word, fpustate = getStatus(FPUGrid[fpu_id])

            
            if FPUGrid[fpu_id].is_collided:
                # only send an error message
                errcode = MCE_WARN_COLLISION_DETECTED
            elif FPUGrid[fpu_id].alpha_limit_breach:
                errcode = MCE_WARN_LIMIT_SWITCH_BREACH
            elif FPUGrid[fpu_id].datum_timeout:
                errcode = MCE_ERR_DATUM_TIME_OUT
            elif FPUGrid[fpu_id].alpha_switch_on():
                errcode = MCE_ERR_DATUM_ON_LIMIT_SWITCH # alpha on limit switch, reject datum command
            else:
                if flag_skip_alpha and not flag_skip_beta):
                    errcode = MCE_NOTIFY_DATUM_ALPHA_ONLY
                elif flag_skip_beta (and not flag_skip_alpha):
                    errcode = MCE_NOTIFY_DATUM_BETA_ONLY
                else:                
                    errcode = MCE_FPU_OK

            # we create the header as usual, with one difference: if
            # an arm was datumed sucessfully, the transmitted step count
            # must be the datum residual error (datum aberration)
            seqnum = RX[0]            if (errcode == MCE_FPU_OK) or errcode == MCE_NOTIFY_DATUM_ALPHA_ONLY) :
                count_alpha = fold_stepcount_alpha(FPUGrid[fpu_id].self.alpha_deviation)
            else:
                count_alpha = fold_stepcount_alpha(FPUGrid[fpu_id].self.alpha_steps)
                
            if (errcode == MCE_FPU_OK) or errcode == MCE_NOTIFY_DATUM_ALPHA_ONLY) :
                count_beta = fold_stepcount_beta(FPUGrid[fpu_id].self.beta_deviation)
            else:
                count_beta = fold_stepcount_beta(FPUGrid[fpu_id].self.beta_steps)

            command_id = CMSG_FINISHED_DATUM
            header_fields=(HDR_SEQNUM | HDR_COMMAND_ID | HDR_STWORD 
                           | HDR_FPUSTATE | HDR_ECODE) # without stepcounts
            
            TX = create_CANheader(command_id, fpu_id, seqnum, errcode, fields=header_fields)

            
            if (errcode == MCE_FPU_OK) or errcode == MCE_NOTIFY_DATUM_ALPHA_ONLY) :
                count_alpha = fold_stepcount_alpha(FPUGrid[fpu_id].self.alpha_deviation)
            else:
                count_alpha = fold_stepcount_alpha(FPUGrid[fpu_id].self.alpha_steps)
                
            if (errcode == MCE_FPU_OK) or errcode == MCE_NOTIFY_DATUM_ALPHA_ONLY) :
                count_beta = fold_stepcount_beta(FPUGrid[fpu_id].self.beta_deviation)
            else:
                count_beta = fold_stepcount_beta(FPUGrid[fpu_id].self.beta_steps)
            
            TX[4] = count0 = count_alpha & 0xff
            TX[5] = count1 = (dev_alpha >> 8) & 0xff
    
            TX[6] = count2 = count_beta & 0xff
            TX[7] = count3 = (count_beta >> 8) & 0xff

            
            finish_message =  TH + TX

            
            #print("FPU %i: findDatum command finished" % fpu_id);
            encode_and_send(finish_message, socket, verbose=opts.debug)

        # "spawn_later" inserts a timed event into the aynchronous event loop
        # - similar to a thread but not running in parallel.
        spawn_later(1, findDatum_func, fpu_id, fpu_adr_bus, bus_adr, RX, socket, opts)
    
    # the new state goes into the header
    TH = create_gwheader(fpu_adr_bus, bus_adr)
    TX = create_CANheader(command_id, fpu_id, seqnum, errcode)            

    status_word, fpustate = getStatus(FPUGrid[fpu_id])
    
    ## send confirmation message 
    #print("FPU %i: sending confirmation to findDatum command" % fpu_id);
    confirmation_message = TH + TX

    return confirmation_message


        
##############################

def handle_executeMotion(fpu_id, fpu_adr_bus, bus_adr, RX, socket, verbose=False):
        
    print("starting executeMotion for FPU %i" % fpu_id)

    if len(RX) < 8:
        print("CAN command format error, length must be 8");
        return []

    ## gateway header
    
    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpu_adr_bus
    
    TH = [ 0 ] * 3
    TH[0] = bus_adr
    TH[1] = (tx_canid & 0xff)
    TH[2] = ((tx_canid >> 8) & 0xff)
    
    command_id = RX[0]

    TX = [0] * 8
    TX[0] = fpu_adr_bus
    TX[1] = command_id
    TX[2] = getStatus(FPUGrid[fpu_id]) 
    
    TX[5] = dummy0 = 0
    TX[6] = dummy1 = 0
    TX[7] = dummy2 = 0

    
    if FPUGrid[fpu_id].is_collided:
        # collision active, only send an error message
        TX[3] = errflag = 0xff
        TX[4] = errcode = ER_COLLIDE
    elif not FPUGrid[fpu_id].wave_ready:
        # wavetable is not ready
        print("FPU #", fpu_id, ": wave table is not ready, sending response code", ER_WAVENRDY)
        TX[3] = errflag = 0xff
        TX[4] = errcode =  ER_WAVENRDY
    elif not FPUGrid[fpu_id].wave_valid:
        # wavetable is not ready
        print("FPU #", fpu_id, ": wave table is not valid, sending response code", ER_INVALID)
        TX[3] = errflag = 0xff
        TX[4] = errcode =  ER_INVALID
    elif FPUGrid[fpu_id].running_wave:
        # FPU already moving
        TX[3] = errflag = 0xff
        TX[4] = errcode = ER_INVALID        
    else:
        # all OK, send confirmation and spawn executeMotion method call
        TX[2] = TX[2] | STBT_RUNNING_WAVE
        TX[3] = errflag = 0


        def executeMotion_func(fpu_id, fpu_adr_bus, bus_adr, RX, socket, verbose=False):

            tx_prio = 0x02
            tx_canid = (tx_prio << 7) | fpu_adr_bus
            
            TH = [ 0 ] * 3
            TH[0] = bus_adr
            TH[1] = (tx_canid & 0xff)
            TH[2] = ((tx_canid >> 8) & 0xff)

            command_id = RX[0]
            
            

            
            # instantiate callbacks
            limit_callback = LimitCallback(fpu_adr_bus, bus_adr, socket)
            collision_callback = CollisionCallback(fpu_adr_bus, bus_adr, socket)
            
            # simulate findDatum FPU operation
            FPUGrid[fpu_id].executeMotion(sleep, limit_callback.call, collision_callback.call)
            print("FPU %i: executeMotion command finished" % fpu_id);

            
            TX = [0] * 8
            TX[0] = fpu_adr_bus
            TX[1] = CMSG_FINISHED_MOTION
            TX[2] = status = getStatus(FPUGrid[fpu_id])
            
            if FPUGrid[fpu_id].is_collided:
                # only send an error message
                TX[3] = errflag = 0xff
                TX[4] = errcode = ER_COLLIDE
            elif status & STBT_M1LIMIT:
                TX[3] = errflag = 0xff
                TX[4] = errcode = ER_M1LIMIT
            if FPUGrid[fpu_id].step_timing_fault:
                # send error message
                TX[3] = errflag = 0xff
                TX[4] = errcode = ER_TIMING
            else:
                # in version 1, other cases do not have
                # status flag information
                TX[3] = errflag = 0
                TX[4] = dummy0 = 0
                
            TX[5] = dummy1 = 0
            TX[6] = dummy2 = 0
            TX[7] = dummy3 = 0
            
            #print("FPU %i: findDatum command finished" % fpu_id);
            finish_message =  TH + TX
            encode_and_send(finish_message, socket, verbose=verbose)

        # "spawn" inserts a event into the aynchronous event loop
        # - similar to a thread but not running in parallel.
        spawn(executeMotion_func, fpu_id, fpu_adr_bus, bus_adr, RX, socket, verbose=verbose)
    
    ## send confirmation message 
    #print("FPU %i: sending confirmation to findDatum command" % fpu_id);
    confirmation_message = TH + TX

    return confirmation_message


def handle_repeatMotion(fpu_id, fpu_adr_bus, bus_adr, RX):
    command_id = RX[0]

    # CAN header for gateway
    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpu_adr_bus
    
    TH = [ 0 ] * 3
    TH[0] = bus_adr
    TH[1] = (tx_canid & 0xff)
    TH[2] = ((tx_canid >> 8) & 0xff)

    TX = [ 0 ] * 8
    
    TX[0] = fpu_adr_bus
    TX[1] = command_id


    if not FPUGrid[fpu_id].wave_valid:
        # wavetable is not ready
        print("FPU #", fpu_id, ": wave table is not valid, sending response code", ER_INVALID)
        TX[3] = errflag = 0xff
        TX[4] = errcode =  ER_INVALID
    else:
        try:
            FPUGrid[fpu_id].repeatMotion(fpu_id)
            TX[3] = errflag = 0
            
        except RunTimeError:
            TX[3] = errflag = 0xff
            TX[4] = errcode = ER_INVALID
            
    TX[2] = getStatus(FPUGrid[fpu_id])
    
    return TH + TX


def handle_reverseMotion(fpu_id, fpu_adr_bus, bus_adr, RX):
    command_id = RX[0]

    # CAN header for gateway
    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpu_adr_bus
    
    TH = [ 0 ] * 3
    TH[0] = bus_adr
    TH[1] = (tx_canid & 0xff)
    TH[2] = ((tx_canid >> 8) & 0xff)

    TX = [ 0 ] * 8
    
    TX[0] = fpu_adr_bus
    TX[1] = command_id


    if not FPUGrid[fpu_id].wave_valid:
        # wavetable is not ready
        print("FPU #", fpu_id, ": wave table is not valid, sending response code", ER_INVALID)
        TX[3] = errflag = 0xff
        TX[4] = errcode =  ER_INVALIDY
    else:
        try:
            FPUGrid[fpu_id].reverseMotion(fpu_id)
            TX[3] = errflag = 0
            
        except RunTimeError:
            TX[3] = errflag = 0xff
            TX[4] = errcode = ER_INVALID
            
    TX[2] = getStatus(FPUGrid[fpu_id])
    
    return TH + TX


def handle_readSerialNumber(fpu_id, fpu_adr_bus, bus_adr, RX):

    seqnum = RX[0]
    command_id = RX[1] & 0x1f
    
    TH = create_gwheader(fpu_adr_bus, bus_adr)
    TX = create_CANheader(command_id, fpu_id, seqnum,
                          ecode=MCE_FPU_OK,
                          fields= HDR_SEQNUM | HDR_COMMAND_ID)

    sn = FPUGrid[fpu_id].readSerialNumber()
    assert(len(sn) <= LEN_SERIAL_NUMBER)
    for k in range(len(sn)):
        TX[2+k] = ord(sn[k])

    return TH + TX 

def handle_writeSerialNumber(fpu_id, fpu_adr_bus, bus_adr, RX):

    serial_number = ""
    chrs = RX[2:]
    for c in chrs:
        if c == 0:
            break
        serial_number += chr(c)


    try:
        FPUGrid[fpu_id].writeSerialNumber(serial_number)
        ecode = MCE_FPU_OK
    except RuntimeError:
        ecode = MCE_ERR_INVALID_PARAMETER
    
    seqnum = RX[0]
    command_id = RX[1] & 0x1f
    
    TH = create_gwheader(fpu_adr_bus, bus_adr)
    TX = create_CANheader(command_id, fpu_id, seqnum, ecode)

    
    return TH + TX

def handle_getFirmwareVersion(fpu_id, fpu_adr_bus, bus_adr, RX):

    seqnum = RX[0]
    command_id = RX[1] & 0x1f
    
    TH = create_gwheader(fpu_adr_bus, bus_adr)
    TX = create_CANheader(command_id, fpu_id, seqnum,
                          ecode=MCE_FPU_OK,
                          fields= HDR_SEQNUM | HDR_COMMAND_ID | HDR_STWORD)

    fv_major, fv_minor, fv_patch, fv_year, fv_month, fv_day = FPUGrid[fpu_id].getFirmwareVersion()
    
    TX[3] = fv_patch
    TX[4] = fv_minor
    TX[5] = fv_mayor

    dateval = ((fv_year & 0x7f)
               | ((fv_month  & 0x0f ) << 7)
               | ((fv_day & 0x1f) << 11) )
    TX[6] = dateval & 0xff
    TX[7] = (dateval >> 8) & 0xff

    return TH + TX 


def fpu_handler(command_id, fpu_id, fpu_adr_bus,bus_adr, rx_bytes, socket, args):
    verbose = args.debug
    if fpu_id >= args.NUM_FPUS:
        print("Warning: command sent to non-existant FPU ID #%i (discarded)" % fpu_id)
        return
    
    if command_id == CCMD_PING_FPU                         :
        # resp = handle_pingFPU(fpu_id, cmd)
        resp = handle_pingFPU(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)
        
    elif command_id == CCMD_RESET_FPU                        :
        resp = handle_resetFPU(fpu_id, fpu_adr_bus, bus_adr, rx_bytes, socket, verbose=verbose)
        
    elif command_id == CCMD_FIND_DATUM                       :
        # we pass the socket here to send an interim confirmation
        resp = handle_findDatum(fpu_id, fpu_adr_bus, bus_adr, rx_bytes, socket, args)
        
    elif command_id == CCMD_CONFIG_MOTION  :
        resp = handle_configMotion(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)
        
    elif command_id == CCMD_EXECUTE_MOTION :
        resp = handle_executeMotion(fpu_id, fpu_adr_bus, bus_adr, rx_bytes, socket, verbose=verbose)
        
    elif command_id == CCMD_ABORT_MOTION   :
        resp = handle_abortMotion(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)

    elif command_id == CCMD_READ_REGISTER                    :
        resp = handle_readRegister(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)        

    elif command_id == CCMD_REPEAT_MOTION                    :
        resp = handle_repeatMotion(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)

    elif command_id == CCMD_REVERSE_MOTION                   :
        resp = handle_reverseMotion(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)

    elif command_id == CCMD_ENABLE_BETA_COLLISION_PROTECTION :
        resp = handle_enableBetaCollisionProtection(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)

    elif command_id == CCMD_FREE_BETA_COLLISION              :
        resp = handle_freeBetaCollision(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)

    elif command_id == CCMD_SET_USTEP_LEVEL                  :
        resp = handle_setUStepLevel(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)

    elif command_id == CCMD_READ_SERIAL_NUMBER               :
            resp = handle_readSerialNumber(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)
            
    elif command_id == CCMD_WRITE_SERIAL_NUMBER               :
        resp = handle_writeSerialNumber(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)
            
    elif command_id == CCMD_GET_FIRMWARE_VERSION               :
        resp = handle_getFirmwareVersion(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)
        
    elif command_id == CCMD_RESET_STEPCOUNTER                :
        pass

    elif command_id == CCMD_LOCK_UNIT                        :
        pass
    
    elif command_id == CCMD_UNLOCK_UNIT                      :
        pass
    
    elif command_id == CCMD_GET_FIRMWARE_VERSION             :
        pass
    
    elif command_id == CCMD_CHECK_INTEGRITY                  :
        pass
    
    elif command_id == CCMD_FREE_ALPHA_LIMIT_BREACH          :
        pass
    
    elif command_id == CCMD_ENABLE_ALPHA_LIMIT_PROTECTION    :
        pass
    
    elif command_id == CCMD_SET_TIME_STEP                    :
        pass
    
    elif command_id == CCMD_SET_STEPS_PER_FRAME              :
        pass
    
    elif command_id == CCMD_ENABLE_MOVE                      :
        pass
        
    else:
        resp = handle_invalidCommand(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)

    if resp != None:
        encode_and_send(resp, socket, verbose=verbose)
        #print("echoed %r" % response)



gCountTotalCommands = 0

def command_handler(cmd, socket, args):
    verbose = args.verbosity > 0
    global gCountTotalCommands
    gCountTotalCommands += 1
    if verbose:
        print("command decoded bytes are:", cmd)
    gateway_id = gateway_map[socket.getsockname()]
    bus_adr = cmd[0]
    rx_canid = cmd[1] + (cmd[2] << 8)
    # we now have the sequence number in cmd[3] alias RX[0]
    command_id = cmd[4]
    bus_global_id = bus_adr + gateway_id * BUSES_PER_GATEWAY

    
    if bus_adr == MSG_TYPE_DELY:
        rx_bytes = cmd[3:]
        delay = rx_bytes[0]
        if args.verbosity > 0:
            print("CAN gateway delay command [count %i] to gw %i, bus %i: delay %i ms"
                  % (gCountTotalCommands, gateway_id, bus_adr, delay))
        
        
    elif rx_canid != 0:
        # non-broadcast message
        rx_priority = (rx_canid >> 7)
        fpu_adr_bus = rx_canid & 0x7f # this is a one-based index
        fpu_id = (fpu_adr_bus-1) + bus_global_id * FPUS_PER_BUS
        
        rx_bytes = cmd[3:]
        if verbose:
            print("CAN command [count %i] to gw %i, bus %i, fpu # %i (rx_priority %i), command id=%i"
                  % (gCountTotalCommands, gateway_id, bus_adr, fpu_adr_bus, rx_priority, command_id))
            
            print("CAN command #%i to FPU %i" % (command_id, fpu_id))


        fpu_handler(command_id, fpu_id, fpu_adr_bus,bus_adr, rx_bytes, socket, args)
    else:
        # broadcast message
        if verbose:
            print("CAN BROADCAST command [%i] to gw %i, bus %i, command id=%i"
                  % (gCountTotalCommands, gateway_id, bus_adr, command_id))

        rx_bytes = cmd[3:]
        for fpu_adr_bus in range(1, FPUS_PER_BUS+1):
            fpu_id = (fpu_adr_bus-1) + bus_global_id * FPUS_PER_BUS
            assert(fpu_id >= 0)
            if fpu_id < args.NUM_FPUS:

                #print("Spawning CAN command #%i to FPU %i" % (command_id, fpu_id))
                spawn(fpu_handler, command_id, fpu_id, fpu_adr_bus, bus_adr, rx_bytes, socket, args)
            



if __name__ == "__main__":
    print("run mock_gateway to use this module.")

