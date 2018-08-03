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

#  number of buses on one gateway
BUSES_PER_GATEWAY =  5
# number of FPUs on one CAN bus
FPUS_PER_BUS = 76

CAN_PROTOCOL_VERSION = 1

# command codes

CCMD_NO_COMMAND                       = 0 # reserved
CCMD_CONFIG_MOTION                    = 1 # configure waveform
CCMD_EXECUTE_MOTION                   = 2 # execute loaded waveform
CCMD_ABORT_MOTION                     = 3 # abort any ongoing movement
    

CCMD_READ_REGISTER                    = 6 # read register
CCMD_PING_FPU                         = 7 # check connectivity
CCMD_RESET_FPU                        = 8 # reset MCU
CCMD_FIND_DATUM                       = 9 # "automatic" datum search
CCMD_RESET_STEPCOUNTER                = 10 # only for debugging
CCMD_REPEAT_MOTION                    = 11 # re-use last waveform
CCMD_REVERSE_MOTION                   = 12 # invert last waveform
CCMD_ENABLE_BETA_COLLISION_PROTECTION = 13 # "ENABLE_COLLIDE"
CCMD_FREE_BETA_COLLISION              = 14 # "FREE_COLLIDE"
CCMD_SET_USTEP_LEVEL                  = 15 # set motor micro-stepping (1,2,4,8 supported)

if CAN_PROTOCOL_VERSION == 1:
    # In version 2, two are covered by the ping command, which also
    # reports the current positions.
    CCMD_GET_STEPS_ALPHA                  = 4 # get alpha counts
    CCMD_GET_STEPS_BETA                   = 5 # get beta counts
    # the next two are combined in version 2
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
else:
    CCMD_LOCK_UNIT                        = 4 # ignore any command except reset and unlock
    CCMD_UNLOCK_UNIT                      = 5 # listen to commands again

    # commands which are not yet implemented
    CCMD_GET_COUNTER_DEVIATION            = 16 # get alpha and beta residue count    
    CCMD_GET_FIRMWARE_VERSION             = 17 # get firmware version
    CCMD_CHECK_INTEGRITY                  = 18 # report firmware CRC
    CCMD_FREE_ALPHA_LIMIT_BREACH          = 19 # untangle alpha arm
    CCMD_ENABLE_ALPHA_LIMIT_PROTECTION    = 20 # re-enable limit switch
    CCMD_SET_TIME_STEP                    = 21 # set movement time interval
    CCMD_SET_STEPS_PER_FRAME              = 22 # set minimum step frequency
    CCMD_ENABLE_MOVE                      = 23 # set minimum step frequency
    CCMD_READ_SERIAL_NUMBER               = 24 # read serial number from NVRAM
    CCMD_WRITE_SERIAL_NUMBER              = 25 # write serial number to NVRAM
                                          
    CMSG_FINISHED_MOTION                  = 26 # executeMotion finished
    CMSG_FINISHED_DATUM                   = 27 # findDatum finished
    CMSG_WARN_COLLISION_BETA              = 28 # collision at beta arm
    CMSG_WARN_LIMIT_ALPHA                 = 29 # limit switch at alpha arm
    CMSG_WARN_TIMEOUT_DATUM               = 30 # datum search time out
    
    NUM_CAN_COMMANDS = 30


# error codes
ER_STALLX           = 0x01        #  x motor stall (obsolete)
ER_STALLY           = 0x02        #  y motor stall (obsolete)
ER_COLLIDE          = 0x03        #  FPU collision detected
ER_INVALID          = 0x04        #  received command not valid
ER_WAVENRDY         = 0x05        #  waveform not ready 
ER_WAVE2BIG         = 0x06        #  waveform exceeds memory allocation
ER_TIMING           = 0x07        #  step timing error (interrupt race condition)
ER_M1LIMIT          = 0x08        #  M1 Limit switch breached
ER_M2LIMIT          = 0x09        #  no longer used
ER_PARAM            = 0x10        #  parameter out of range
ER_AUTO             = 0x11        #  FPU cannot datum automatically
ER_DATUMTO          = 0x12        #  hardware error: datum search timed out by firmware
ER_DATUM_LIMIT      = 0x13        # datum search denied, limit switch is active
# (note: the driver uses additional internal status codes)


# status flags
STBT_MSGRCV          = 0        #  message received over CANBUS
STBT_WAVE_READY      = 1 << 1   #  waveform good and ready for execution
STBT_EXECUTE_WAVE    = 1 << 2   #  internal start flag to start executing waveform
STBT_RUNNING_WAVE    = 1 << 3   #  FPU is running the waveform
STBT_ABORT_WAVE      = 1 << 4   #  abort waveform
STBT_M1LIMIT         = 1 << 5   #  M1 Limit breached 
STBT_M2LIMIT         = 1 << 6   #  no longer used
STBT_REVERSE_WAVE    = 1 << 7   #  waveform to be run in reverse

# datum option flags

DATUM_SKIP_ALPHA = 1
DATUM_SKIP_BETA = (1 << 1)
DATUM_MODE_AUTO = (1 << 2)
DATUM_MODE_ANTI_CLOCKWISE = (1 << 3)
DATUM_TIMEOUT_DISABLE = (1 << 4)

def encode_and_send(msg, socket, verbose=False):
    confirmation = codec.encode(msg, verbose=verbose)
    socket.sendall(confirmation)


def fold_stepcount_alpha(val):
    low_limit = - 10000
    vrange = 1 << 16
    assert( (val >= low_limit) and (val < (low_limit + vrange)))
    # convert to unsigned 16-bit number
            
    if val < 0:
        val += (1 << 16)

    return val

def fold_stepcount_beta(val):
    low_limit = - 0x8000
    vrange = 1 << 16
    assert( (val >= low_limit) and (val < (low_limit + vrange)))
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
    if FPU.wave_ready:
        status |=  STBT_WAVE_READY
        
    if FPU.running_wave:
        status |= STBT_RUNNING_WAVE

    if FPU.abort_wave:
        status |= STBT_ABORT_WAVE

    if not FPU.move_forward:
        status |= STBT_REVERSE_WAVE
        
    if FPU.alpha_limit_breach:
        status |=  STBT_M1LIMIT
        
    return status
        
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
        
        
    if verbose:
        print("FPU #%i command =%i , rx=%s" % (fpu_id, command_id, RX))
    
    try:
        FPUGrid[fpu_id].addStep(first_entry, last_entry,
                                astep, apause, aclockwise,
                                bstep, bpause, bclockwise)

        tx3_errflag = 0
        tx4_errcode = 0
    
    except IndexError:
        tx3_errflag = 0xff
        tx4_errcode = ER_PARAM
    except RuntimeError:
        tx3_errflag = 0xff
        tx4_errcode = ER_INVALID


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
        TX[4] = tx4_errcode
        TX[5] = dummy1 = 0
        TX[6] = dummy2 = 0
        TX[7] = dummy3 = 0
        
        confirmation = TH + TX 
        return confirmation
    else:
        # no confirmation
        return None


def handle_GetX(fpu_id, fpu_adr_bus, bus_adr, RX):

    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpu_adr_bus
    
    TH = [ 0 ] * 3
    TH[0] = bus_adr
    TH[1] = (tx_canid & 0xff)
    TH[2] = ((tx_canid >> 8) & 0xff)

    TX = [0] * 8

    command_id = RX[0]

    TX[0] = fpu_adr_bus
    TX[1] = command_id
    TX[2] = getStatus(FPUGrid[fpu_id]) 
    TX[3] = errflag = 0

    pos = fold_stepcount_alpha(FPUGrid[fpu_id].alpha_steps)
    TX[4] = count0 = pos & 0xff
    TX[5] = count1 = (pos >> 8) & 0xff
    # protocol changed here
    TX[6] = _count2 = 0

    TX[7] = dummy = 0
    
    return TH + TX 


def handle_GetY(fpu_id, fpu_adr_bus, bus_adr, RX):

    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpu_adr_bus
    
    TH = [ 0 ] * 3
    TH[0] = bus_adr
    TH[1] = (tx_canid & 0xff)
    TH[2] = ((tx_canid >> 8) & 0xff)

    TX = [0] * 8
    TX[0] = fpu_adr_bus

    command_id = RX[0]

    TX[0] = tx0_fpu_adr_bus = fpu_adr_bus
    TX[1] = command_id
    TX[2] = getStatus(FPUGrid[fpu_id]) 
    TX[3] = errflag = 0

    pos = fold_stepcount_beta(FPUGrid[fpu_id].beta_steps)
    TX[4] = count0 = pos & 0xff
    TX[5] = count1 = (pos >> 8) & 0xff
    # protocol changed here
    TX[6] = _count2 = 0

    TX[7] = dummy = 0
    
    return TH + TX 


def handle_GetErrorAlpha(fpu_id, fpu_adr_bus, bus_adr, RX):

    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpu_adr_bus
    
    TH = [ 0 ] * 3
    TH[0] = bus_adr
    TH[1] = (tx_canid & 0xff)
    TH[2] = ((tx_canid >> 8) & 0xff)

    TX = [0] * 8
    TX[0] = fpu_adr_bus

    command_id = RX[0]

    TX[0] = tx0_fpu_adr_bus = fpu_adr_bus
    TX[1] = command_id
    TX[2] = getStatus(FPUGrid[fpu_id]) 
    TX[3] = errflag = 0

    pos = fold_stepcount_deviation(FPUGrid[fpu_id].alpha_deviation)
    TX[4] = count0 = pos & 0xff
    TX[5] = count1 = (pos >> 8) & 0xff
    # protocol changed here
    TX[6] = _count2 = 0

    TX[7] = dummy = 0
    
    return TH + TX 

def handle_GetErrorBeta(fpu_id, fpu_adr_bus, bus_adr, RX):

    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpu_adr_bus
    
    TH = [ 0 ] * 3
    TH[0] = bus_adr
    TH[1] = (tx_canid & 0xff)
    TH[2] = ((tx_canid >> 8) & 0xff)

    TX = [0] * 8
    TX[0] = fpu_adr_bus

    command_id = RX[0]

    TX[0] = tx0_fpu_adr_bus = fpu_adr_bus
    TX[1] = command_id
    TX[2] = getStatus(FPUGrid[fpu_id]) 
    TX[3] = errflag = 0

    pos = fold_stepcount_deviation(FPUGrid[fpu_id].beta_deviation)
    TX[4] = count0 = pos & 0xff
    TX[5] = count1 = (pos >> 8) & 0xff
    # protocol changed here
    TX[6] = _count2 = 0

    TX[7] = dummy = 0
    
    return TH + TX 



def handle_pingFPU(fpu_id, fpu_adr_bus, bus_adr, RX):
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
    TX[2] = getStatus(FPUGrid[fpu_id])
    TX[3] = errflag = 0

    pos_alpha = fold_stepcount_alpha(FPUGrid[fpu_id].alpha_steps)
    TX[4] = count0 = pos_alpha & 0xff
    TX[5] = count1 = (pos_alpha >> 8) & 0xff
    
    pos_beta = fold_stepcount_beta(FPUGrid[fpu_id].beta_steps)
    TX[6] = count2 = pos_beta & 0xff
    TX[7] = count3 = (pos_beta >> 8) & 0xff

    
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
        command_id = RX[0]

        FPUGrid[fpu_id].resetFPU(fpu_id, sleep);

        ## gateway header
        tx_prio = 0x02
        tx_canid = (tx_prio << 7) | fpu_adr_bus
    
        TH = [ 0 ] * 3
        TH[0] = bus_adr
        TH[1] = (tx_canid & 0xff)
        TH[2] = ((tx_canid >> 8) & 0xff)


        # response message packet
        TX = [0] * 8
        TX[0] = fpu_adr_bus
        TX[1] = command_id
        TX[2] = getStatus(FPUGrid[fpu_id])
        TX[3] = errflag = 0        
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
    
    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpu_adr_bus
    
    TH = [ 0 ] * 3
    TH[0] = bus_adr
    TH[1] = (tx_canid & 0xff)
    TH[2] = ((tx_canid >> 8) & 0xff)

    TX = [0] * 8
    TX[0] = fpu_adr_bus
    
    command_id = RX[0]
    TX[1] = command_id
    TX[2] = getStatus(FPUGrid[fpu_id])
    
    TX[5] = dummy0 = 0
    TX[6] = dummy1 = 0
    TX[7] = dummy2 = 0

    flag_skip_alpha = False
    flag_skip_beta = False
    flag_auto_datum = False
    flag_anti_clockwise = False
    flag_disable_timeout = False
    skip_flag = RX[1] # protocol 1 !
    
    if skip_flag > 0:
        if opts.fw_version > (1,0,0):
            flag_skip_alpha = (skip_flag & DATUM_SKIP_ALPHA) > 0
            flag_skip_beta = (skip_flag & DATUM_SKIP_BETA) > 0
            
            if opts.fw_version > (1,1,0):
                flag_auto_datum = (skip_flag & DATUM_MODE_AUTO) > 0
                flag_anti_clockwise = (skip_flag & DATUM_MODE_ANTI_CLOCKWISE) > 0
            else:
                print("WARNING: protocol version %r running, "
                      +"ignoring mode selection flags", opts.fw_version)
                skip_flag = skip_flag & 0x3
        else:
            print("WARNING: protocol version %r running,"
                  + " ignoring arm and mode selection flags flags", opts.fw_version)
            skip_flag = 0
            
    if opts.fw_version >= (1,4,3):
        if (skip_flag & DATUM_TIMEOUT_DISABLE) > 0:
            flag_disable_timeout = True
    
    if FPUGrid[fpu_id].is_collided:
        # only send an error message
        TX[3] = errflag = 0xff
        TX[4] = errcode = ER_COLLIDE
    elif flag_auto_datum and (not FPUGrid[fpu_id].was_initialized):
        TX[3] = errflag = 0xff
        TX[4] = errcode = ER_AUTO # not initialized, reject automatic datum search
    elif (FPUGrid[fpu_id].opts.fw_version >= (1, 4, 0)) and FPUGrid[fpu_id].alpha_switch_on():
        TX[3] = errflag = 0xff
        TX[4] = errcode = ER_DATUM_LIMIT # alpha on limit switch, reject datum command
        TX[5] = 0
    else:
        # send confirmation and spawn findDatum method call
        TX[3] = errflag = 0
        TX[4] = errcode = 0


        def findDatum_func(fpu_id, fpu_adr_bus, bus_adr, RX, socket, opts):

            tx_prio = 0x02
            tx_canid = (tx_prio << 7) | fpu_adr_bus
            
            TH = [ 0 ] * 3
            TH[0] = bus_adr
            TH[1] = (tx_canid & 0xff)
            TH[2] = ((tx_canid >> 8) & 0xff)

            command_id = RX[0]


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

            TX = [0] * 8
            TX[0] = fpu_adr_bus
            TX[1] = CMSG_FINISHED_DATUM
            TX[2] = status = getStatus(FPUGrid[fpu_id])

            if FPUGrid[fpu_id].is_collided:
                # only send an error message
                TX[3] = errflag = 0xff
                TX[4] = errcode = ER_COLLIDE
                TX[5] = 0
            elif status & STBT_M1LIMIT:
                TX[3] = errflag = 0xff
                TX[4] = errcode = ER_M1LIMIT
                TX[5] = 0
            elif FPUGrid[fpu_id].datum_timeout:
                TX[3] = errflag = 0xff
                TX[4] = errcode = ER_DATUMTO
                TX[5] = 0
            else:
                # in version 1, other cases do not have
                # status flag / error code information
                TX[3] = errflag = 0
                TX[4] = dummy0 = 0
                TX[5] = skip_flag

                
            TX[6] = dummy2 = 0
            TX[7] = dummy3 = 0

            
            finish_message =  TH + TX

            
            #print("FPU %i: findDatum command finished" % fpu_id);
            encode_and_send(finish_message, socket, verbose=opts.debug)

        # "spawn_later" inserts a timed event into the aynchronous event loop
        # - similar to a thread but not running in parallel.
        spawn_later(1, findDatum_func, fpu_id, fpu_adr_bus, bus_adr, RX, socket, opts)
    
    ## send confirmation message 
    #print("FPU %i: sending confirmation to findDatum command" % fpu_id);
    conf_msg = TH + TX

    return conf_msg


        
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
        TX[4] = errcode =  ER_INVALIDY
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

    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpu_adr_bus
    
    TH = [ 0 ] * 3
    TH[0] = bus_adr
    TH[1] = (tx_canid & 0xff)
    TH[2] = ((tx_canid >> 8) & 0xff)

    TX = [0] * 8
    TX[0] = fpu_adr_bus

    command_id = RX[0]

    TX[0] = tx0_fpu_adr_bus = fpu_adr_bus
    TX[1] = command_id
    TX[2] = getStatus(FPUGrid[fpu_id])
    sn = FPUGrid[fpu_id].readSerialNumber()
    assert(len(sn) <= 5)
    for k in range(len(sn)):
        TX[3+k] = ord(sn[k])


    return TH + TX 

def handle_writeSerialNumber(fpu_id, fpu_adr_bus, bus_adr, RX):
    command_id = RX[0]

    # CAN header for gateway
    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpu_adr_bus
    
    TH = [ 0 ] * 3
    TH[0] = bus_adr
    TH[1] = (tx_canid & 0xff)
    TH[2] = ((tx_canid >> 8) & 0xff)

    serial_number = ""
    chrs = RX[1:6]
    for c in chrs:
        if c == 0:
            break
        serial_number += chr(c)


    try:
        FPUGrid[fpu_id].writeSerialNumber(serial_number)
        errflag = 0
        ecode = 0
    except RuntimeError:
        errflag = 0xff
        ecode = 1
    
    TX = [ 0 ] * 8
    
    TX[0] = fpu_adr_bus
    TX[1] = command_id
    TX[2] = getStatus(FPUGrid[fpu_id])
    TX[3] = errflag
    TX[4] = ecode


    
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
        

    elif command_id == CCMD_RESET_STEPCOUNTER                :
        pass

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

    elif CAN_PROTOCOL_VERSION == 1:
        if command_id == CCMD_GET_STEPS_ALPHA:
            resp = handle_GetX(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)
            
        elif command_id == CCMD_GET_STEPS_BETA:
            resp = handle_GetY(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)
            
        elif command_id == CCMD_GET_ERROR_ALPHA                  :
            resp = handle_GetErrorAlpha(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)
            
        elif command_id == CCMD_GET_ERROR_BETA                   :
            resp = handle_GetErrorBeta(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)
            
        elif command_id == CCMD_READ_SERIAL_NUMBER               :
            resp = handle_readSerialNumber(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)
            
        elif command_id == CCMD_WRITE_SERIAL_NUMBER               :
            resp = handle_writeSerialNumber(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)
            
        else:
            resp = handle_invalidCommand(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)
    else:
        if command_id == CCMD_LOCK_UNIT                        :
            pass
        elif command_id == CCMD_UNLOCK_UNIT                      :
            pass
        elif command_id == CCMD_GET_COUNTER_DEVIATION            :
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
        elif command_id == CCMD_READ_SERIAL_NUMBER               :
            pass            
        elif command_id == CCMD_WRITE_SERIAL_NUMBER               :
            pass
        else:
            resp = handle_invalidCommand(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)

    if resp != None:
        encode_and_send(resp, socket, verbose=verbose)
        #print("echoed %r" % response)



gCountTotalCommands = 0

def command_handler(cmd, socket, args):
    verbose = args.debug
    global gCountTotalCommands
    gCountTotalCommands += 1
    if verbose:
        print("command decoded bytes are:", cmd)
    gateway_id = gateway_map[socket.getsockname()]
    bus_adr = cmd[0]
    rx_canid = cmd[1] + (cmd[2] << 8)
    command_id = cmd[3]
    bus_global_id = bus_adr + gateway_id * BUSES_PER_GATEWAY

    if rx_canid != 0:
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

