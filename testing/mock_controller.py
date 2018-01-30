#!/usr/bin/python

"""
This is a mock FPU grid controller which decodes and
synthesizes a response.

"""

from __future__ import print_function
import codec
from fpu_sim import FPU

from gevent import sleep, spawn, spawn_later

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


def handle_configMotion(fpu_id, cmd):
    bus_adr = cmd[0]
    canid = cmd[1] + (cmd[2] << 8)
    fpu_adr_bus = canid & 0x7f # this is a one-based index
    rx_priority = (canid >> 7)
    command_id = cmd[3]

    tx_bus_adr = bus_adr
    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpu_adr_bus
    tx0_fpu_adr_bus = fpu_adr_bus
    tx1_cmdid = command_id
    tx2_status = 0
    tx3_errflag = 0

    rx = cmd[3:]
    first_entry = rx[1] & 1
    if first_entry:
        print("first_entry set!")
    last_entry = (rx[1] >> 1) & 1
    if last_entry:
        print("last_entry set!")
    
    astep = ((rx[3] &  0x3f) << 8) + rx[2]
    apause = (rx[3] >> 6) & 1
    aclockwise = (rx[3] >> 7) & 1

    bstep = ((rx[5] &  0x3f) << 8) + rx[4]
    bpause = (rx[5] >> 6) & 1
    bclockwise = (rx[5] >> 7) & 1

    print("FPU #%i command = " % fpu_id, cmd)
    try:
        FPUGrid[fpu_id].addStep(first_entry, last_entry,
                                astep, apause, aclockwise,
                                bstep, bpause, bclockwise)

        if FPUGrid[fpu_id].wave_ready:
            tx2_status = STBT_WAVE_READY
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

    if first_entry or last_entry:
        confirmation = [ tx_bus_adr,
             (tx_canid & 0xff),
             ((tx_canid >> 8) & 0xff),
             tx0_fpu_adr_bus,
             tx1_cmdid,
             tx2_status,
             tx3_errflag,
             tx4_dummy,
             tx5_dummy,
             tx6_dummy ,
             tx7_dummy ]
        return confirmation
    else:
        # no confirmation
        return None


def handle_GetX(fpu_id, cmd):
    bus_adr = cmd[0]
    canid = cmd[1] + (cmd[2] << 8)
    fpu_adr_bus = canid & 0x7f # this is a one-based index
    rx_priority = (canid >> 7)
    command_id = cmd[3]

    tx_bus_adr = bus_adr
    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpu_adr_bus
    tx0_fpu_adr_bus = fpu_adr_bus
    tx1_cmdid = command_id
    tx2_status = 0
    tx3_errflag = 0

    pos = fold_stepcount_alpha(FPUGrid[fpu_id].alpha_steps)
    tx4_count0 = pos & 0xff
    tx5_count1 = (pos >> 8) & 0xff
    # protocol changed here
    tx6_count2 = 0

    tx7_dummy = 0
    
    return [ tx_bus_adr,
             (tx_canid & 0xff),
             ((tx_canid >> 8) & 0xff),
             tx0_fpu_adr_bus,
             tx1_cmdid,
             tx2_status,
             tx3_errflag,
             tx4_count0,
             tx5_count1,
             tx6_count2,
             tx7_dummy ]

def handle_findDatum(fpu_id, cmd, socket, verbose=False):
        
    print("starting findDatum for FPU %i" % fpu_id)

    if len(cmd) < 8:
        print("CAN command format error, length must be 8");
        return []
    
    bus_adr = cmd[0]
    canid = cmd[1] + (cmd[2] << 8)
    fpu_adr_bus = canid & 0x7f # this is a one-based index
    rx_priority = (canid >> 7)
    command_id = cmd[3]

    tx_bus_adr = bus_adr
    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpu_adr_bus
    tx0_fpu_adr_bus = fpu_adr_bus
    
    tx4_dummy0 = 0
    tx5_dummy1 = 0
    tx6_dummy2 = 0
    tx7_dummy3 = 0

    if FPUGrid[fpu_id].is_collided:
        tx1_cmdid = command_id
        tx2_status = 0xff
        tx3_errflag = ER_COLLIDE
    else:
        tx1_cmdid = command_id 
        tx2_status = 0
        tx3_errflag = 0


        def findDatum_func(fpu_id, cmd, socket, verbose=False):

            bus_adr = cmd[0]
            canid = cmd[1] + (cmd[2] << 8)
            fpu_adr_bus = canid & 0x7f # this is a one-based index
            rx_priority = (canid >> 7)
            command_id = cmd[3]
            
            tx_bus_adr = bus_adr
            tx_prio = 0x02
            tx_canid = (tx_prio << 7) | fpu_adr_bus
            tx0_fpu_adr_bus = fpu_adr_bus
            
            tx4_dummy0 = 0
            tx5_dummy1 = 0
            tx6_dummy2 = 0
            tx7_dummy3 = 0

            

            # simulate findDatum FPU operation
            FPUGrid[fpu_id].findDatum(sleep)

    
            tx1_cmdid = CMSG_FINISHED_DATUM
    
            finish_message =  [ tx_bus_adr,
                                (tx_canid & 0xff),
                                ((tx_canid >> 8) & 0xff),
                                tx0_fpu_adr_bus,
                                tx1_cmdid,
                                tx2_status,
                                tx3_errflag,
                                tx4_dummy0,
                                tx5_dummy1,
                                tx6_dummy2,
                                tx7_dummy3 ]
            
            #print("FPU %i: findDatum command finished" % fpu_id);
            encode_and_send(finish_message, socket, verbose=verbose)

        spawn_later(1, findDatum_func, fpu_id, cmd, socket, verbose=verbose)
    
    ## send confirmation message 
    #print("FPU %i: sending confirmation to findDatum command" % fpu_id);
    conf_msg = [ tx_bus_adr,
             (tx_canid & 0xff),
             ((tx_canid >> 8) & 0xff),
             tx0_fpu_adr_bus,
             tx1_cmdid,
             tx2_status,
             tx3_errflag,
             tx4_dummy0,
             tx5_dummy1,
             tx6_dummy2,
             tx7_dummy3 ]

    return conf_msg


def handle_GetY(fpu_id, cmd):
    bus_adr = cmd[0]
    canid = cmd[1] + (cmd[2] << 8)
    fpu_adr_bus = canid & 0x7f # this is a one-based index
    rx_priority = (canid >> 7)
    command_id = cmd[3]

    tx_bus_adr = bus_adr
    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpu_adr_bus
    tx0_fpu_adr_bus = fpu_adr_bus
    tx1_cmdid = command_id
    tx2_status = 0
    tx3_errflag = 0

    pos = fold_stepcount_beta(FPUGrid[fpu_id].beta_steps)
    tx4_count0 = pos & 0xff
    tx5_count1 = (pos >> 8) & 0xff
    tx6_count2 = 0

    tx7_dummy = 0
    
    return [ tx_bus_adr,
             (tx_canid & 0xff),
             ((tx_canid >> 8) & 0xff),
             tx0_fpu_adr_bus,
             tx1_cmdid,
             tx2_status,
             tx3_errflag,
             tx4_count0,
             tx5_count1,
             tx6_count2,
             tx7_dummy ]

def handle_PingFPU(fpu_id, fpu_adr_bus, bus_adr, RX):
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
    TX[2] = status = 0
    TX[3] = errflag = 0

    pos_alpha = fold_stepcount_alpha(FPUGrid[fpu_id].alpha_steps)
    TX[4] = count0 = pos_alpha & 0xff
    TX[5] = count1 = (pos_alpha >> 8) & 0xff
    
    pos_beta = fold_stepcount_beta(FPUGrid[fpu_id].beta_steps)
    TX[6] = count2 = pos_beta & 0xff
    TX[7] = count3 = (pos_beta >> 8) & 0xff

    
    return TH + TX



def handle_resetFPU(fpu_id, cmd, socket, verbose=False):

    def reset_func(fpu_id, cmd, socket, verbose=False):
        bus_adr = cmd[0]
        canid = cmd[1] + (cmd[2] << 8)
        fpu_adr_bus = canid & 0x7f # this is a one-based index
        rx_priority = (canid >> 7)
        command_id = cmd[3]

        FPUGrid[fpu_id].resetFPU(fpu_id, sleep);

        tx_bus_adr = bus_adr
        tx_prio = 0x02
        tx_canid = (tx_prio << 7) | fpu_adr_bus


    
        tx0_fpu_adr_bus = fpu_adr_bus
        tx1_cmdid = command_id
        tx2_status = 0
        tx3_errflag = 0
        
        tx4_count0 = 0
        tx5_count1 = 0
    
        tx6_count2 = 0
        tx7_count3 = 0

        conf_msg = [ tx_bus_adr,
             (tx_canid & 0xff),
             ((tx_canid >> 8) & 0xff),
             tx0_fpu_adr_bus,
             tx1_cmdid,
             tx2_status,
             tx3_errflag,
             tx4_count0,
             tx5_count1,
             tx6_count2,
             tx7_count3 ]

        print("fpu #%i: sending reset confirmation" % fpu_id)
        encode_and_send(conf_msg, socket, verbose=verbose)

    spawn(reset_func, fpu_id, cmd, socket, verbose=verbose)
        
    print("returning from reset message")
    
    return None

def handle_invalidCommand(fpu_id, cmd):
    
    bus_adr = cmd[0]
    canid = cmd[1] + (cmd[2] << 8)
    fpu_adr_bus = canid & 0x7f # this is a one-based index
    rx_priority = (canid >> 7)
    command_id = cmd[3]

    tx_bus_adr = bus_adr
    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpu_adr_bus
    tx0_fpu_adr_bus = fpu_adr_bus
    tx1_cmdid = command_id
    tx2_status = 0
    tx3_errflag = 0xff
    tx4_errcode = ER_INVALID

    tx5_dummy = 0
    tx6_dummy = 0
    tx7_dummy = 0
    
    return [ tx_bus_adr,
             (tx_canid & 0xff),
             ((tx_canid >> 8) & 0xff),
             tx0_fpu_adr_bus,
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
    bus_adr = cmd[0]
    canid = cmd[1] + (cmd[2] << 8)
    rx_priority = (canid >> 7)
    fpu_adr_bus = canid & 0x7f # this is a one-based index
    command_id = cmd[3]
    bus_global_id = bus_adr + gateway_id * BUSES_PER_GATEWAY
    fpu_id = (fpu_adr_bus-1) + bus_global_id * FPUS_PER_BUS

    if verbose:
        print("CAN command [%i] to gw %i, bus %i, fpu # %i (rx_priority %i), command id=%i"
          % (gCountTotalCommands, gateway_id, bus_adr, fpu_adr_bus, rx_priority, command_id))
    
        print("CAN command #%i to FPU %i" % (command_id, fpu_id))



    if command_id == CCMD_PING_FPU                         :
        # resp = handle_PingFPU(fpu_id, cmd)
        resp = handle_PingFPU(fpu_id, fpu_adr_bus, bus_adr, cmd[3:])
        
    elif command_id == CCMD_RESET_FPU                        :
        resp = handle_resetFPU(fpu_id, cmd, socket, verbose=verbose)
        
    elif command_id == CCMD_FIND_DATUM                       :
        # we pass the socket here to send an interim confirmation
        resp = handle_findDatum(fpu_id, cmd, socket, verbose=verbose)
        
    elif command_id == CCMD_CONFIG_MOTION  :
        resp = handle_configMotion(fpu_id, cmd)        
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
            resp = handle_invalidCommand(fpu_adr_bus, cmd)
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
            resp = handle_invalidCommand(fpu_adr_bus, cmd)

    if resp != None:
        encode_and_send(resp, socket, verbose=verbose)
        #print("echoed %r" % response)


if __name__ == "__main__":
    print("run mock_gateway to use this module.")

