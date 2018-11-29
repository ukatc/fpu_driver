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

from protocol_constants import *

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


def getStatus(fpu):
    status = 0
    if fpu.alpha_datum_active:
        status |=  STBT_ALPHA_DATUM_ACTIVE
        
    if fpu.beta_datum_active:
        status |=  STBT_BETA_DATUM_ACTIVE
        
    if fpu.is_collided:
        status |=  STBT_COLLISION_DETECTED
        
    if fpu.alpha_limit_breach:
        print("alpha limit breach flag is set")
        status |= STBT_ALPHA_AT_LIMIT 
        
    if fpu.state == FPST_LOCKED:
        status |= STBT_FPU_LOCKED
        
    # the following two fields are for recovery porposes - they
    # don't have a meaningful value before the first movement
    # has started (we could use a third state, but this isn't worth the
    # extra bits which are expensive)
    if fpu.alpha_last_direction == DIRST_CLOCKWISE:
        status |=  ALPHA_LAST_DIRECTION
        
    if fpu.beta_last_direction == DIRST_CLOCKWISE:
        status |=  beta_LAST_DIRECTION

    # alpha and beta state are lumped together here -
    # the common case, extra info goes in findDatum status code
    if fpu.was_initialized:
        status |=  STBT_IS_ZEROED
        
    if fpu.wave_ready:
        status |=  STBT_WAVEFORM_READY
        
    if fpu.wave_valid:
        status |=  STBT_WAVEFORM_VALID
        
    if fpu.wave_reversed:
        status |= STBT_WAVEFORM_REVERSED
                
    return (status, fpu.state )


def create_gwheader(fpu_adr_bus, bus_adr, command_id):
    # CAN header for gateway
    tx_prio = priorities_by_cmd_id.get(command_id, DEFAULT_PRIORITY)
    assert(tx_prio <= 0b1111)
    
    tx_canid = (tx_prio << 7) | fpu_adr_bus
    
    TH = [ 0 ] * 3
    TH[0] = bus_adr
    TH[1] = (tx_canid & 0xff)
    TH[2] = ((tx_canid >> 8) & 0xff)

    return TH



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

    status_word, fpu_state = getStatus(FPUGrid[fpu_id])
    if fields & HDR_SEQNUM:
        TX[0] = seqnum
        
    if fields & HDR_COMMAND_ID:
        TX[1] = command_id & 0x1f
        
    if fields & HDR_STWORD:
        TX[1] = TX[1] |  ( ( 0x7 & status_word) << 5)
        TX[2] = 0xff & ( status_word >> 3)
        
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

    return TX
        
def handle_configMotion(fpu_id, fpu_adr_bus, bus_adr, RX, verbosity=0):

    seqnum = RX[0]
    command_id = RX[1] & 0x1f
    
    first_entry = RX[2] & 1
    last_entry = (RX[2] >> 1) & 1
    send_response = (RX[2] >> 2) & 1
    
    if first_entry and verbosity > 2:
        print("handle_configMotion(): first_entry set!")
    if last_entry and verbosity > 2:
        print("handle_configMotion(): last_entry set!")
    if send_response and verbosity > 2:
        print("handle_configMotion(): response rqeusted!")

    apause = (RX[4] >> 6) & 1
    astep = ((RX[4] &  0x3f) << 8) + RX[3]
    if apause:
        astep = 0
    aclockwise = (RX[4] >> 7) & 1

    bpause = (RX[6] >> 6) & 1
    bstep = ((RX[6] &  0x3f) << 8) + RX[5]
    if bpause:
        bstep=0
    bclockwise = (RX[6] >> 7) & 1
        
    nwave_entries = 0
    
    if verbosity > 3:
        print("FPU #%i command =%i , rx=%s" % (fpu_id, command_id, RX))

        
    errcode, wf_errcode = FPUGrid[fpu_id].addStep(first_entry, last_entry,
                                                  astep, apause, aclockwise,
                                                  bstep, bpause, bclockwise)

    print("addStep: result is %r" % ( (errcode, wf_errcode), ))
    

    if errcode in [ MCE_ERR_CAN_OVERFLOW_HW, MCE_ERR_CAN_OVERFLOW_SW ] :
        command_id = CMSG_WARN_CANOVERFLOW
        

    send_confirmation = (first_entry
                         or last_entry
                         or send_response
                         or (command_id == CMSG_WARN_CANOVERFLOW))
        
    if send_confirmation:        
        TH = create_gwheader(fpu_adr_bus, bus_adr, command_id)
        TX = create_CANheader(command_id, fpu_id, seqnum, errcode)
        
        if command_id == CCMD_CONFIG_MOTION:
            TX[4] = FPUGrid[fpu_id].nwave_entries
            TX[5] = wf_errcode
        
        confirmation = TH + TX[:6]
        return confirmation
    else:
        return None



def handle_pingFPU(fpu_id, fpu_adr_bus, bus_adr, RX):
    seqnum = RX[0]
    command_id = RX[1] & 0x1f
    
    TH = create_gwheader(fpu_adr_bus, bus_adr, command_id)
    TX = create_CANheader(command_id, fpu_id, seqnum, ecode=MCE_FPU_OK)
    
    return TH + TX


def handle_abortMotion(fpu_id, fpu_adr_bus, bus_adr, RX):


    errcode = FPUGrid[fpu_id].abortMotion(fpu_id)

    seqnum = RX[0]
    command_id = RX[1] & 0x1f
    
    TH = create_gwheader(fpu_adr_bus, bus_adr, command_id)
    TX = create_CANheader(command_id, fpu_id, seqnum, errcode)    
    
    return TH + TX


def handle_freeBetaCollision(fpu_id, fpu_adr_bus, bus_adr, RX):


    print("mock_controller: freeBetaCollision")
    direction = RX[2]
    errcode = FPUGrid[fpu_id].freeBetaCollision(direction)
    
    seqnum = RX[0]
    command_id = RX[1] & 0x1f
    
    TH = create_gwheader(fpu_adr_bus, bus_adr, command_id)
    TX = create_CANheader(command_id, fpu_id, seqnum, errcode)
    
    return TH + TX


def handle_enableBetaCollisionProtection(fpu_id, fpu_adr_bus, bus_adr, RX):
    
    errcode = FPUGrid[fpu_id].enableBetaCollisionProtection()
    
    seqnum = RX[0]
    command_id = RX[1] & 0x1f
    TH = create_gwheader(fpu_adr_bus, bus_adr, command_id)
    TX = create_CANheader(command_id, fpu_id, seqnum, errcode)
    
    return TH + TX


def handle_freeAlphaLimitBreach(fpu_id, fpu_adr_bus, bus_adr, RX):

    direction = RX[1]

    errcode = FPUGrid[fpu_id].freeAlphaLimitBreach(direction)
    
    seqnum = RX[0]
    command_id = RX[1] & 0x1f
    
    TH = create_gwheader(fpu_adr_bus, bus_adr, command_id)
    TX = create_CANheader(command_id, fpu_id, seqnum, errcode)

    
    return TH + TX


def handle_enableAlphaLimitBreachProtection(fpu_id, fpu_adr_bus, bus_adr, RX):
    
    errcode = FPUGrid[fpu_id].enableAlphaLimitBreachProtection()
    
    seqnum = RX[0]
    command_id = RX[1] & 0x1f
    TH = create_gwheader(fpu_adr_bus, bus_adr, command_id)
    TX = create_CANheader(command_id, fpu_id, seqnum, errcode)
    
    return TH + TX


def handle_setUStepLevel(fpu_id, fpu_adr_bus, bus_adr, RX):


    ustep_level = RX[2]
    errcode = FPUGrid[fpu_id].setUStepLevel(ustep_level)

    seqnum = RX[0]
    command_id = RX[1] & 0x1f
    
    TH = create_gwheader(fpu_adr_bus, bus_adr, command_id)
    TX = create_CANheader(command_id, fpu_id, seqnum, errcode)
    
    return TH + TX


def handle_setStepsPerSegment(fpu_id, fpu_adr_bus, bus_adr, RX):

    min_steps_per_segment = (RX[2] << 8) | RX[3]
    max_steps_per_segment = (RX[4] << 8) | RX[5]

    errcode = FPUGrid[fpu_id].setStepsPerFrame(min_steps_per_frame, max_steps_per_frame)

    seqnum = RX[0]
    command_id = RX[1] & 0x1f
    
    TH = create_gwheader(fpu_adr_bus, bus_adr, command_id)
    TX = create_CANheader(command_id, fpu_id, seqnum, errcode)
    
    return TH + TX


def handle_setTicksPerSegment(fpu_id, fpu_adr_bus, bus_adr, RX):

    ticks_per_frame = RX[2] | (RX[3] << 8) | (RX[4] << 16)

    errcode = FPUGrid[fpu_id].setTicksPerSegment(ticks_per_frame)

    seqnum = RX[0]
    command_id = RX[1] & 0x1f
    
    TH = create_gwheader(fpu_adr_bus, bus_adr, command_id)
    TX = create_CANheader(command_id, fpu_id, seqnum, errcode)
    
    return TH + TX



def handle_readRegister(fpu_id, fpu_adr_bus, bus_adr, RX):

    seqnum = RX[0]
    command_id = RX[1] & 0x1f
    
    register_address = ((RX[2] << 8) | RX[3])

    
    byte = FPUGrid[fpu_id].getRegister(register_address)
    
    print("fpu #%i: read from address 0x%04x yields 0x%02x" %
          (fpu_id,register_address, byte) )
    
    
    TH = create_gwheader(fpu_adr_bus, bus_adr, command_id)
    TX = create_CANheader(command_id, fpu_id, seqnum, MCE_FPU_OK)
    TX[4] = (register_address >> 8) & 0xff
    TX[5] = register_address & 0xff
    TX[6] = byte
    
    return TH + TX[:7]


def handle_resetFPU(fpu_id, fpu_adr_bus, bus_adr, RX, socket, verbose=False):

    def reset_func(fpu_id, fpu_adr_bus, bus_adr, RX, socket, verbose=False):

        FPUGrid[fpu_id].resetFPU(fpu_id, sleep);

        # response message packet        
        seqnum = RX[0]
        command_id = RX[1] & 0x1f
    
        TH = create_gwheader(fpu_adr_bus, bus_adr, command_id)
        TX = create_CANheader(command_id, fpu_id, seqnum, ecode=MCE_FPU_OK)

        conf_msg = TH + TX 

        print("fpu #%i: sending reset confirmation" % fpu_id)
        encode_and_send(conf_msg, socket, verbose=verbose)

    spawn(reset_func, fpu_id, fpu_adr_bus, bus_adr, RX, socket, verbose=verbose)
        
    print("returning from reset message")
    
    return None


def handle_resetStepCounter(fpu_id, fpu_adr_bus, bus_adr, RX, socket, verbose=False):


    errcode = FPUGrid[fpu_id].resetStepCounter(fpu_id);

    # response message packet        
    seqnum = RX[0]
    command_id = RX[1] & 0x1f
    
    TH = create_gwheader(fpu_adr_bus, bus_adr, command_id)
    TX = create_CANheader(command_id, fpu_id, seqnum, errcode)
    
    return TH + TX 


def handle_lockUnit(fpu_id, fpu_adr_bus, bus_adr, RX):
    
    errcode = FPUGrid[fpu_id].lockUnit()
    
    seqnum = RX[0]
    command_id = RX[1] & 0x1f
    TH = create_gwheader(fpu_adr_bus, bus_adr, command_id)
    TX = create_CANheader(command_id, fpu_id, seqnum, errcode)
    
    return TH + TX

def handle_unlockUnit(fpu_id, fpu_adr_bus, bus_adr, RX):
    
    errcode = FPUGrid[fpu_id].unlockUnit()
    
    seqnum = RX[0]
    command_id = RX[1] & 0x1f
    TH = create_gwheader(fpu_adr_bus, bus_adr, command_id)
    TX = create_CANheader(command_id, fpu_id, seqnum, errcode)
    
    return TH + TX

def handle_enableMove(fpu_id, fpu_adr_bus, bus_adr, RX):
    
    errcode = FPUGrid[fpu_id].enableMove()
    
    seqnum = RX[0]
    command_id = RX[1] & 0x1f
    TH = create_gwheader(fpu_adr_bus, bus_adr, command_id)
    TX = create_CANheader(command_id, fpu_id, seqnum, errcode)
    
    return TH + TX

def handle_checkIntegrity(fpu_id, fpu_adr_bus, bus_adr, RX):
    
    crc32val = FPUGrid[fpu_id].checkIntegrity()
    
    seqnum = RX[0]
    command_id = RX[1] & 0x1f
    TH = create_gwheader(fpu_adr_bus, bus_adr, command_id)
    TX = create_CANheader(command_id, fpu_id, seqnum, errcode,
                          fields = (HDR_SEQNUM | HDR_COMMAND_ID | HDR_STWORD 
                                    | HDR_FPUSTATE | HDR_ECODE) )
    for k in range(4):
        TX[4 + k] = 0xff & crc32val
        crc32val = crc32val >> 8
    
    return TH + TX

 
def handle_invalidCommand(fpu_id, fpu_adr_bus, bus_adr, RX):
    
    seqnum = RX[0]
    command_id = RX[1] & 0x1f
    errcode = MCE_ERR_INVALID_COMMAND
    
    TH = create_gwheader(fpu_adr_bus, bus_adr, command_id)
    TX = create_CANheader(command_id, fpu_id, seqnum, errcode)
    
    return TH + TX 



##############################
# the following two callback objects are passed as bound methods
# to the FPUs executeMotion and findDatum methods so that FPUs can signal
# collisions or limit switch breaks

class LimitCallback:
    def __init__(self, fpu_id, fpu_adr_bus, bus_adr, socket, seqnum, verbose=False):
        self.socket = socket
        self.fpu_id = fpu_id
        self.fpu_adr_bus = fpu_adr_bus
        self.bus_adr = bus_adr
        self.verbose=verbose
        self.seqnum = seqnum
        
    def call(self, fpu):

        tx_prio = 0x01
        
        TH = create_gwheader(self.fpu_adr_bus, self.bus_adr, CMSG_WARN_LIMIT_ALPHA)
        TX = create_CANheader(CMSG_WARN_LIMIT_ALPHA, self.fpu_id, self.seqnum, MCE_WARN_LIMIT_SWITCH_BREACH)

        if self.verbose:
            print("FPU %i: sending limit switch break message" % self.fpu_id)
            
        limit_message =  TH + TX
        encode_and_send(limit_message, self.socket, verbose=self.verbose)

class CollisionCallback:
    def __init__(self, fpu_id, fpu_adr_bus, bus_adr, socket, seqnum, verbose=False):
        self.socket = socket
        self.fpu_id = fpu_id
        self.fpu_adr_bus = fpu_adr_bus
        self.bus_adr = bus_adr
        self.verbose = verbose
        self.seqnum = seqnum
        
    def call(self, fpu):

        tx_prio = 0x01
        
        if self.verbose:
            print("FPU %i: sending collision detection message" % self.fpu_id)
    
        TH = create_gwheader(self.fpu_adr_bus, self.bus_adr, MCE_WARN_COLLISION_DETECTED)
        TX = create_CANheader(CMSG_WARN_COLLISION_BETA, self.fpu_id, self.seqnum, MCE_WARN_COLLISION_DETECTED)
            
        limit_message =  TH + TX
        encode_and_send(limit_message, self.socket, verbose=self.verbose)


#################################

def handle_findDatum(fpu_id, fpu_adr_bus, bus_adr, RX, socket, opts):
        
    print("starting findDatum for FPU %i" % fpu_id)

    if len(RX) > 8:
        print("CAN command format error, length must be not larger than 8");
        return []

    ## gateway header
    
    seqnum = RX[0]
    command_id = RX[1] & 0x1f
    
    TH = create_gwheader(fpu_adr_bus, bus_adr, command_id)
    TX = create_CANheader(command_id, fpu_id, seqnum, MCE_FPU_OK)
    
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


    fpu = FPUGrid[fpu_id]
    errcode = fpu.start_findDatum(flag_auto_datum)

    if errcode == MCE_FPU_OK:

        # create closure which sends return message when operation is finished
        # (non-local variables are looked up in the enclosing scope)
        
        def findDatum_func(fpu_id, fpu_adr_bus, bus_adr, RX, socket, opts):

            # instantiate two objects which can send collision messages
            # if needed
            limit_callback = LimitCallback(fpu_id, fpu_adr_bus, bus_adr, socket, seqnum)
            collision_callback = CollisionCallback(fpu_id, fpu_adr_bus, bus_adr, socket, seqnum)


            # simulate findDatum FPU operation
            errcode = fpu.findDatum(sleep,
                                    limit_callback.call, collision_callback.call,
                                    skip_alpha=flag_skip_alpha,
                                    skip_beta=flag_skip_beta,
                                    auto_datum=flag_auto_datum,
                                    anti_clockwise=flag_anti_clockwise,
                                    disable_timeout=flag_disable_timeout)
            
            print("FPU %i: findDatum command finished" % fpu_id);

    
            command_id = CMSG_FINISHED_DATUM
            
            TH = create_gwheader(fpu_adr_bus, bus_adr, command_id)

        
            header_fields=(HDR_SEQNUM | HDR_COMMAND_ID | HDR_STWORD 
                           | HDR_FPUSTATE | HDR_ECODE) # without stepcounts
            
            TX = create_CANheader(command_id, fpu_id, seqnum, errcode, fields=header_fields)

                        
            # we create the message body as usual, with one difference: if
            # an arm was datumed sucessfully, the transmitted step count
            # must be the datum residual error (datum aberration)

            if (errcode == MCE_FPU_OK) or (errcode == MCE_NOTIFY_DATUM_ALPHA_ONLY) :
                count_alpha = fold_stepcount_alpha(fpu.alpha_deviation)
            else:
                count_alpha = fold_stepcount_alpha(fpu.alpha_steps)
                
            if (errcode == MCE_FPU_OK) or (errcode == MCE_NOTIFY_DATUM_BETA_ONLY) :
                count_beta = fold_stepcount_beta(fpu.beta_deviation)
            else:
                count_beta = fold_stepcount_beta(fpu.beta_steps)
                
            TX[4] = count0 = count_alpha & 0xff
            TX[5] = count1 = (count_alpha >> 8) & 0xff
    
            TX[6] = count2 = count_beta & 0xff
            TX[7] = count3 = (count_beta >> 8) & 0xff

            
            finish_message =  TH + TX

            
            #print("FPU %i: findDatum command finished" % fpu_id);
            encode_and_send(finish_message, socket, verbose=opts.debug)

        # "spawn_later" inserts a timed event into the aynchronous event loop
        # - similar to a thread but not running in parallel.
        spawn_later(1, findDatum_func, fpu_id, fpu_adr_bus, bus_adr, RX, socket, opts)
    
    # the new state goes into the header
    TH = create_gwheader(fpu_adr_bus, bus_adr, command_id)
    TX = create_CANheader(command_id, fpu_id, seqnum, errcode)            

    status_word, fpustate = getStatus(FPUGrid[fpu_id])
    
    ## send confirmation message 
    #print("FPU %i: sending confirmation to findDatum command" % fpu_id);
    confirmation_message = TH + TX

    return confirmation_message


        
##############################

def handle_executeMotion(fpu_id, fpu_adr_bus, bus_adr, RX, socket, verbose=False):
        
    print("starting executeMotion for FPU %i" % fpu_id)

    if len(RX) > 8:
        print("CAN command format error, length must be equal or smaller 8");
        return []

    seqnum = RX[0]
    command_id = RX[1] & 0x1f
    

    errcode = FPUGrid[fpu_id].start_executeMotion()

    if errcode == MCE_FPU_OK:
        
        # all OK, send confirmation and spawn executeMotion method call

        def executeMotion_func(fpu_id, fpu_adr_bus, bus_adr, RX, socket, verbose=False):

            # instantiate callbacks
            limit_callback = LimitCallback(fpu_id, fpu_adr_bus, bus_adr, socket, seqnum)
            collision_callback = CollisionCallback(fpu_id, fpu_adr_bus, bus_adr, socket, seqnum)
            
            # simulate executeMotion FPU operation
            errcode = FPUGrid[fpu_id].executeMotion(sleep, limit_callback.call, collision_callback.call)
            
            print("FPU %i: executeMotion command finished with error code %i" % (fpu_id, errcode));
                               
            TH = create_gwheader(fpu_adr_bus, bus_adr, CMSG_FINISHED_MOTION)
            TX = create_CANheader(CMSG_FINISHED_MOTION, fpu_id, seqnum, errcode)
            
            finish_message =  TH + TX
            encode_and_send(finish_message, socket, verbose=verbose)

        # "spawn" inserts a event into the aynchronous event loop
        # - similar to a thread but not running in parallel.
        spawn(executeMotion_func, fpu_id, fpu_adr_bus, bus_adr, RX, socket, verbose=verbose)
    
    TH = create_gwheader(fpu_adr_bus, bus_adr, command_id)
    TX = create_CANheader(command_id, fpu_id, seqnum, errcode)
    
    ## send confirmation message 
    confirmation_message = TH + TX

    return confirmation_message


def handle_repeatMotion(fpu_id, fpu_adr_bus, bus_adr, RX):

    errcode = FPUGrid[fpu_id].repeatMotion(fpu_id)
            
    seqnum = RX[0]
    command_id = RX[1] & 0x1f
    
    TH = create_gwheader(fpu_adr_bus, bus_adr, command_id)
    TX = create_CANheader(command_id, fpu_id, seqnum, MCE_FPU_OK)
    
    
    return TH + TX


def handle_reverseMotion(fpu_id, fpu_adr_bus, bus_adr, RX):

    errcode = FPUGrid[fpu_id].reverseMotion(fpu_id)
            
    seqnum = RX[0]
    command_id = RX[1] & 0x1f
    
    TH = create_gwheader(fpu_adr_bus, bus_adr, command_id)
    TX = create_CANheader(command_id, fpu_id, seqnum, MCE_FPU_OK)

    
    return TH + TX


def handle_readSerialNumber(fpu_id, fpu_adr_bus, bus_adr, RX):

    seqnum = RX[0]
    command_id = RX[1] & 0x1f
    
    TH = create_gwheader(fpu_adr_bus, bus_adr, command_id)
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
    
    TH = create_gwheader(fpu_adr_bus, bus_adr, command_id)
    TX = create_CANheader(command_id, fpu_id, seqnum, ecode)

    
    return TH + TX

def handle_getFirmwareVersion(fpu_id, fpu_adr_bus, bus_adr, RX):

    seqnum = RX[0]
    command_id = RX[1] & 0x1f
    
    TH = create_gwheader(fpu_adr_bus, bus_adr, command_id)
    TX = create_CANheader(command_id, fpu_id, seqnum,
                          ecode=MCE_FPU_OK,
                          fields= HDR_SEQNUM | HDR_COMMAND_ID | HDR_STWORD)

    fv_major, fv_minor, fv_patch, fv_year, fv_month, fv_day = FPUGrid[fpu_id].getFirmwareVersion()
    
    TX[3] = fv_major
    TX[4] = fv_minor
    TX[5] = fv_patch

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
        resp = handle_configMotion(fpu_id, fpu_adr_bus, bus_adr, rx_bytes, verbosity=args.verbosity)
        
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

    elif command_id == CCMD_ENABLE_ALPHA_LIMIT_PROTECTION :
        resp = handle_enableAlphaLimitBreachProtection(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)

    elif command_id == CCMD_FREE_ALPHA_LIMIT_BREACH              :
        resp = handle_freeAlphaLimitBreach(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)

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
        
    elif command_id == CCMD_LOCK_UNIT                        :
        resp = handle_lockUnit(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)
    
    elif command_id == CCMD_UNLOCK_UNIT                      :
        resp = handle_unlockUnit(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)
    
    elif command_id == CCMD_CHECK_INTEGRITY                  :
        resp = handle_checkIntegrity(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)
    
    elif command_id == CCMD_SET_TICKS_PER_SEGMENT                   :
        resp = handle_setTicksPerSegment(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)
    
    elif command_id == CCMD_SET_STEPS_PER_SEGMENT              :
        resp = handle_setStepsPerSegment(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)
    
    elif command_id == CCMD_ENABLE_MOVE                      :
        resp = handle_enableMove(fpu_id, fpu_adr_bus, bus_adr, rx_bytes)
    
    elif command_id == CCMD_RESET_STEPCOUNTER                :
        resp = handle_resetStepCounter(fpu_id, fpu_adr_bus, bus_adr, rx_bytes, socket, verbose=verbose)

        
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

    
    if bus_adr == MSG_TYPE_DELY:
        rx_bytes = cmd[3:]
        delay = rx_bytes[0]
        if args.verbosity > 5:
            print("CAN gateway delay command [count %i] to gw %i, bus %i: delay %i ms"
                  % (gCountTotalCommands, gateway_id, bus_adr, delay))
        
        
    else:
        command_id = cmd[4]
        bus_global_id = bus_adr + gateway_id * BUSES_PER_GATEWAY
        rx_bytes = cmd[3:]
        if rx_canid != 0:
            # we now have the sequence number in cmd[3] alias RX[0]
            
            # non-broadcast message
            rx_priority = (rx_canid >> 7)
            fpu_adr_bus = rx_canid & 0x7f # this is a one-based index
            fpu_id = (fpu_adr_bus-1) + bus_global_id * FPUS_PER_BUS
            

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
    
            for fpu_adr_bus in range(1, FPUS_PER_BUS+1):
                fpu_id = (fpu_adr_bus-1) + bus_global_id * FPUS_PER_BUS
                assert(fpu_id >= 0)
                if fpu_id < args.NUM_FPUS:
    
                    #print("Spawning CAN command #%i to FPU %i" % (command_id, fpu_id))
                    spawn(fpu_handler, command_id, fpu_id, fpu_adr_bus, bus_adr, rx_bytes, socket, args)
                



if __name__ == "__main__":
    print("run mock_gateway to use this module.")

