#!/usr/bin/python

"""
This is a mock FPU grid controller which decodes and
synthesizes a response.

"""

from __future__ import print_function
import codec
from fpu_sim import FPU

from gevent import sleep

#  number of buses on one gateway
BUSES_PER_GATEWAY =  5
# number of FPUs on one CAN bus
FPUS_PER_BUS = 67

CAN_PROTOCOL_VERSION = 1

# command codes

CCMD_NO_COMMAND                       = 0 # reserved
CCMD_CONFIG_MOTION                    = 1 # configure waveform
CCMD_EXECUTE_MOTION                   = 2 # execute loaded waveform
CCMD_ABORT_MOTION                     = 3 # abort any ongoing movement
    

CCMD_READ_REGISTER                    = 6 # read register - unused
CCMD_PING_FPU                         = 7 # check connectivity
CCMD_RESET_FPU                        = 8 # reset MCU
CCMD_FIND_DATUM                       = 9 # "automatic" datum search
CCMD_RESET_STEPCOUNTER                = 10 # only for debugging
CCMD_REPEAT_MOTION                    = 11 # re-use last waveform
CCMD_REVERSE_MOTION                   = 12 # invert last waveform
CCMD_ENABLE_BETA_COLLISION_PROTECTION = 13 # "ENABLE_COLLIDE"
CCMD_FREE_BETA_COLLISION              = 14 # "FREE_COLLIDE"
CCMD_SET_USTEP                        = 15 # set motor micro-stepping (1,2,4,8 supported)

if CAN_PROTOCOL_VERSION == 1:
    # In version 2, two are covered by the ping command, which also
    # reports the current positions.
    CCMD_GET_STEPS_ALPHA                  = 4 # get alpha counts
    CCMD_GET_STEPS_BETA                   = 5 # get beta counts
    # the next two are combined in version 2
    CCMD_GET_ERROR_ALPHA                  = 16 # get residue count at last datum hit
    CCMD_GET_ERROR_BETA                   = 17 # get residue count at last datum hit
    
    NUM_CAN_COMMANDS = 18
    
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
    CCMD_SET_STEPS_PER_FRAME               = 22 # set minimum step frequency
    CCMD_ENABLE_MOVE                      = 23 # set minimum step frequency

    CMSG_FINISHED_MOTION               = 23 # executeMotion finished
    CMSG_FINISHED_DATUM                = 24 # findDatum finished
    CMSG_WARN_COLLISION_BETA           = 25 # collision at beta arm
    CMSG_WARN_LIMIT_ALPHA              = 26 # limit switch at alpha arm
    CMSG_WARN_TIMEOUT_DATUM            = 27 # datum search time out
    
    NUM_CAN_COMMANDS = 24


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


# status flags
STBT_MSGRCV          = 0   #  message received over CANBUS
STBT_WAVE_READY      = 1   #  waveform good and ready for execution
STBT_EXECUTE_WAVE    = 2   #  internal start flag to start executing waveform
STBT_RUNNING_WAVE    = 3   #  FPU is running the waveform
STBT_ABORT_WAVE      = 4   #  abort waveform
STBT_M1LIMIT         = 5   #  M1 Limit breached 
STBT_M2LIMIT         = 6   #  no longer used
STBT_REVERSE_WAVE    = 7   #  waveform to be run in reverse

NUM_FPUS = 15 * 67

FPUGrid = [FPU(i) for i in range(NUM_FPUS) ]

def handle_ConfigMotion(fpu_id, cmd):
    busid = cmd[0]
    canid = cmd[1] + (cmd[2] << 8)
    fpu_busid = canid & 0x7f # this is a one-based index
    priority = (canid >> 7)
    command_id = cmd[3]

    tx_busid = busid
    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpu_busid
    tx0_fpu_busid = fpu_busid
    tx1_cmdid = command_id
    tx2_status = 0
    tx3_errflag = 0

    first_entry = cmd[1] & 1
    last_entry = (cmd[1] >> 1) & 1
    
    astep = ((cmd[3] &  0x3f) << 8) + cmd[2]
    apause = (cmd[3] >> 6) & 1
    aclockwise = (cmd[3] >> 7) & 1

    bstep = ((cmd[5] &  0x3f) << 8) + cmd[4]
    bpause = (cmd[5] >> 6) & 1
    bclockwise = (cmd[5] >> 7) & 1

    try:
        FPUGrid[fpu_id].addStep(first_entry, last_entry,
                                astep, apause, aclockwise,
                                bstep, bpause, bclockwise)

        if FPUGrid[fpu_id].wave_ready:
            tx2_status = WAVE_READY
        else:
            tx2_status = 0
    
    except IndexError:
        tx2_status = 0xff
        tx3_errflag = ER_PARAM
    except RuntimeError:
        tx2_status = 0xff
        tx3_errflag = ER_INVALID

    tx4_dummy = 0
    tx5_dummy = 0
    tx6_dummy = 0
    tx7_dummy = 0
    
    return [ tx_busid,
             (tx_canid & 0xff),
             ((tx_canid >> 8) & 0xff),
             tx0_fpu_busid,
             tx1_cmdid,
             tx2_status,
             tx3_errflag,
             tx4_dummy,
             tx5_dummy,
             tx6_dummy ,
             tx7_dummy ]


def handle_GetX(fpu_id, cmd):
    busid = cmd[0]
    canid = cmd[1] + (cmd[2] << 8)
    fpu_busid = canid & 0x7f # this is a one-based index
    priority = (canid >> 7)
    command_id = cmd[3]

    tx_busid = busid
    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpu_busid
    tx0_fpu_busid = fpu_busid
    tx1_cmdid = command_id
    tx2_status = 0
    tx3_errflag = 0

    pos = FPUGrid[fpu_id].alpha_steps
    tx4_count0 = pos & 0xff
    tx5_count1 = (pos >> 8) & 0xff
    tx6_count2 = (pos >> 16) & 0xff

    tx7_dummy = 0
    
    return [ tx_busid,
             (tx_canid & 0xff),
             ((tx_canid >> 8) & 0xff),
             tx0_fpu_busid,
             tx1_cmdid,
             tx2_status,
             tx3_errflag,
             tx4_count0,
             tx5_count1,
             tx6_count2,
             tx7_dummy ]

def handle_findDatum(fpu_id, cmd):
        
    print("starting findDatum for FPU %i" % fpu_id)
    
    busid = cmd[0]
    canid = cmd[1] + (cmd[2] << 8)
    fpu_busid = canid & 0x7f # this is a one-based index
    priority = (canid >> 7)
    command_id = cmd[3]

    tx_busid = busid
    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpu_busid
    tx0_fpu_busid = fpu_busid
    if FPUGrid[fpu_id].is_collided:
        tx1_cmdid = command_id
        tx2_status = 0xff
        tx3_errflag = ER_COLLIDE
    else:
        FPUGrid[fpu_id].findDatum(sleep)
        tx1_cmdid = CMSG_FINISHED_DATUM
        tx2_status = 0
        tx3_errflag = 0

    tx4_dummy0 = 0
    tx5_dummy1 = 0
    tx6_dummy2 = 0

    tx7_dummy3 = 0
    
    print("responding findDatum for FPU %i" % fpu_id)
    
    return [ tx_busid,
             (tx_canid & 0xff),
             ((tx_canid >> 8) & 0xff),
             tx0_fpu_busid,
             tx1_cmdid,
             tx2_status,
             tx3_errflag,
             tx4_dummy0,
             tx5_dummy1,
             tx6_dummy2,
             tx7_dummy3 ]


def handle_GetY(fpu_id, cmd):
    busid = cmd[0]
    canid = cmd[1] + (cmd[2] << 8)
    fpu_busid = canid & 0x7f # this is a one-based index
    priority = (canid >> 7)
    command_id = cmd[3]

    tx_busid = busid
    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpu_busid
    tx0_fpu_busid = fpu_busid
    tx1_cmdid = command_id
    tx2_status = 0
    tx3_errflag = 0

    pos = FPUGrid[fpu_id].beta_steps
    tx4_count0 = pos & 0xff
    tx5_count1 = (pos >> 8) & 0xff
    tx6_count2 = (pos >> 16) & 0xff

    tx7_dummy = 0
    
    return [ tx_busid,
             (tx_canid & 0xff),
             ((tx_canid >> 8) & 0xff),
             tx0_fpu_busid,
             tx1_cmdid,
             tx2_status,
             tx3_errflag,
             tx4_count0,
             tx5_count1,
             tx6_count2,
             tx7_dummy ]


def handle_invalidCommand(fpu_id, cmd):
    
    busid = cmd[0]
    canid = cmd[1] + (cmd[2] << 8)
    fpu_busid = canid & 0x7f # this is a one-based index
    priority = (canid >> 7)
    command_id = cmd[3]

    tx_busid = busid
    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpu_busid
    tx0_fpu_busid = fpu_busid
    tx1_cmdid = command_id
    tx2_status = 0
    tx3_errflag = 0xff
    tx4_errcode = ER_INVALID

    tx5_dummy = 0
    tx6_dummy = 0
    tx7_dummy = 0
    
    return [ tx_busid,
             (tx_canid & 0xff),
             ((tx_canid >> 8) & 0xff),
             tx0_fpu_busid,
             tx1_cmdid,
             tx2_status,
             tx3_errflag,
             tx4_errcode,
             tx5_dummy,
             tx6_dummy,
             tx7_dummy ]



gCountTotalCommands = 0

def command_handler(cmd, socket, verbose=0):
    global gCountTotalCommands
    gCountTotalCommands += 1
    if verbose:
        print("command decoded bytes are:", cmd)
    gateway_id = gateway_map[socket.getsockname()]
    busid = cmd[0]
    canid = cmd[1] + (cmd[2] << 8)
    priority = (canid >> 7)
    fpu_busid = canid & 0x7f # this is a one-based index
    command_id = cmd[3]
    busnum = busid + gateway_id * BUSES_PER_GATEWAY
    fpu_id = (fpu_busid-1) + busnum * FPUS_PER_BUS

    if verbose:
        print("CAN command [%i] to gw %i, bus %i, fpu # %i (priority %i), command id=%i"
          % (gCountTotalCommands, gateway_id, busid, fpu_busid, priority, command_id))
    
        print("CAN command #%i to FPU %i" % (command_id, fpu_id))



    if command_id == CCMD_PING_FPU                         :
        pass
    elif command_id == CCMD_RESET_FPU                        :
        pass
    elif command_id == CCMD_FIND_DATUM                       :
        resp = handle_findDatum(fpu_id, cmd)        
    elif command_id == CCMD_CONFIG_MOTION  :
        resp = handle_ConfigMotion(fpu_id, cmd)        
    elif command_id == CCMD_EXECUTE_MOTION :
        pass
    elif command_id == CCMD_ABORT_MOTION   :
        pass        
    elif command_id == CCMD_READ_REGISTER                    :
        pass
    elif command_id == CCMD_RESET_STEPCOUNTER                :
        pass
    elif command_id == CCMD_REPEAT_MOTION                    :
        pass
    elif command_id == CCMD_REVERSE_MOTION                   :
        pass
    elif command_id == CCMD_ENABLE_BETA_COLLISION_PROTECTION :
        pass
    elif command_id == CCMD_FREE_BETA_COLLISION              :
        pass
    elif command_id == CCMD_SET_USTEP                        :
        pass
    elif CAN_PROTOCOL_VERSION == 1:
        if command_id == CCMD_GET_STEPS_ALPHA:
            resp = handle_GetX(fpu_id, cmd)
        elif command_id == CCMD_GET_STEPS_BETA:
            resp = handle_GetY(fpu_id, cmd)
        elif command_id == CCMD_GET_ERROR_ALPHA                  :
            pass
        elif command_id == CCMD_GET_ERROR_BETA                   :
            pass
        else:
            resp = handle_invalidCommand(fpu_busid, cmd)
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
        else:
            resp = handle_invalidCommand(fpu_busid, cmd)
            
    response = codec.encode(resp, verbose=verbose)
    socket.sendall(response)
    #print("echoed %r" % response)


if __name__ == "__main__":
    print("run mock_gateway to use this module.")

