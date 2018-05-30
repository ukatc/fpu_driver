#!/usr/bin/python

from __future__ import print_function, division
import os
from os import path
import errno
import time
import signal

import fpu_driver

from fpu_driver import __version__, CAN_PROTOCOL_VERSION, GatewayAddress,  \
    REQD_ANTI_CLOCKWISE,  REQD_CLOCKWISE, FPUDriverException, MovementError, \
    DASEL_BOTH, DASEL_ALPHA, DASEL_BETA, \
    LOG_ERROR, LOG_INFO, LOG_GRIDSTATE, LOG_DEBUG, LOG_VERBOSE, LOG_TRACE_CAN_MESSAGES, \
    SEARCH_CLOCKWISE, SEARCH_ANTI_CLOCKWISE, SEARCH_AUTO, SKIP_FPU


import fpu_commands as cmds


MOCK_GATEWAY_ADRESS_LIST = [ GatewayAddress("127.0.0.1", p)
                                for p in [4700, 4701, 4702] ]


TEST_GATEWAY_ADRESS_LIST = [ GatewayAddress("192.168.0.10", 4700) ]

DEFAULT_GATEWAY_ADRESS_LIST = MOCK_GATEWAY_ADRESS_LIST


DEFAULT_NUM_FPUS=int(os.environ.get("NUM_FPUS","1005"))

DEFAULT_LOGDIR=os.environ.get("FPU_DRIVER_DEFAULT_LOGDIR","./_logs")
str_loglevel = os.environ.get("FPU_DRIVER_DEFAULT_LOGLEVEL","LOG_TRACE_CAN_MESSAGES")
DEFAULT_LOGLEVEL= {"LOG_ERROR":    LOG_ERROR,    
                   "LOG_INFO":     LOG_INFO,     
                   "LOG_GRIDSTATE":LOG_GRIDSTATE,
                   "LOG_VERBOSE":LOG_VERBOSE,
                   "LOG_DEBUG":    LOG_DEBUG,    
                   "LOG_TRACE_CAN_MESSAGES": LOG_TRACE_CAN_MESSAGES}[str_loglevel]


class SignalHandler(object):
    """Context manager for handling a signal
    while waiting for command completion.
    """

    def __init__(self, sig=signal.SIGINT):
        self.sig = sig

    def __enter__(self):

        self.interrupted = False
        self.released = False

        self.original_handler = signal.getsignal(self.sig)

        def handler(signum, frame):
            self.release()
            self.interrupted = True

        signal.signal(self.sig, handler)

        return self

    def __exit__(self, type, value, tb):
        self.release()

    def release(self):

        if self.released:
            return False

        signal.signal(self.sig, self.original_handler)

        self.released = True

        return True

def get_logname(basename,
                log_dir="",
                timestamp=None):
    
    if timestamp=="ISO8601":
        timestamp= time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())

    filename = basename.format(start_timestamp=timestamp)
    return path.join(log_dir, path.expandvars(path.expanduser(filename)))


def make_logdir(log_dir):
    log_path = path.abspath(path.expandvars(path.expanduser(log_dir)))
    try:
        os.makedirs(log_path, 0o00744)
    except OSError as e:
        if e.errno == errno.EEXIST:
            pass
        else:
            raise
        
    if log_dir != DEFAULT_LOGDIR:
        print("logging to ", log_path)
        
    return log_path

class GridDriver:
    def __init__(self, nfpus=DEFAULT_NUM_FPUS,
                 SocketTimeOutSeconds=20.0,
                 logLevel=DEFAULT_LOGLEVEL,
                 log_dir=DEFAULT_LOGDIR,
                 control_logfile="_{start_timestamp}-fpu_control.log",
                 tx_logfile = "_{start_timestamp}-fpu_tx.log",
                 rx_logfile = "_{start_timestamp}-fpu_rx.log",
                 start_timestamp="ISO8601"):


        config = fpu_driver.GridDriverConfig()
        config.num_fpus = nfpus
        config.SocketTimeOutSeconds = SocketTimeOutSeconds
        
        flags = os.O_CREAT | os.O_APPEND | os.O_WRONLY
        mode = 0o00644

        

        config.logLevel = logLevel

        log_path = make_logdir(log_dir)
        
        config.fd_controllog = os.open(get_logname(control_logfile, 
                 log_dir=log_path, timestamp=start_timestamp), flags, mode)
        config.fd_txlog = os.open(get_logname(tx_logfile, 
                 log_dir=log_path, timestamp=start_timestamp), flags, mode)
        config.fd_rxlog = os.open(get_logname(rx_logfile, 
                 log_dir=log_path, timestamp=start_timestamp), flags, mode)
        
        self.config = config

        self._gd = fpu_driver.GridDriver(config)

    def __del__(self):
        # the following delete is necessary to run destructors!!
        del self._gd
        os.close(self.config.fd_controllog)
        os.close(self.config.fd_txlog)
        os.close(self.config.fd_rxlog)
        

    def connect(self, address_list=DEFAULT_GATEWAY_ADRESS_LIST):
        return self._gd.connect(address_list)

    def setUStepLevel(self, ustep_level,  gs):
        return self._gd.setUStepLevel(ustep_level, gs)
    
    def getGridState(self):
        return self._gd.getGridState()

    def findDatumB(self, gs, fpu_modes={}, selected_arm=DASEL_BOTH, check_protection=True):
        """Moves all FPUs to datum position. 

        This is a blocking variant of the findDatum command,
        it is not interruptible by Control-C."""
        
        return self._gd.findDatum(gs, fpu_modes, selected_arm, check_protection)

    def findDatum(self, gs, fpu_modes={}, selected_arm=DASEL_BOTH, check_protection=True):
        """Moves all FPUs to datum position. 

        If the program receives a SIGNINT, or Control-C is pressed, an
        abortMotion command is automatically sended, aborting the search.

        The parameter selected_arm (DASEL_BOTH, DASEL_ALPHA, DASEL_BETA) 
        controls which arms are moved.

        The dictionary fpu_modes has integer FPU IDs as keys, and 
        each value is one of SEARCH_CLOCKWISE, SEARCH_ANTI_CLOCKWISE, SEARCH_AUTO, SKIP_FPU,
        which controls whether the datum search moves clockwise (decreasing step count),
        anti-clockwise (increasing step count), automatically, or skips the FPU.
        The default mode is automatically.

        If an beta arm is not datumed, automatic datum search will be refused.
        If a beta arm position appears not to be safe to be moved into
        the requested position, the manual datum search will be refused
        unless check_protection is set to False.

        """
        rv = self._gd.startFindDatum(gs, fpu_modes, selected_arm, check_protection)
        if rv != fpu_driver.E_DriverErrCode.DE_OK:
            raise RuntimeError("can't search Datum, driver error code = %r" % rv)
        time.sleep(0.1)
        time_interval = 0.1
        is_ready = False
        was_aborted = False
        with SignalHandler() as sh:
            while not is_ready:
                rv = self._gd.waitFindDatum(gs, time_interval)
                if sh.interrupted:
                    print("STOPPING FPUs.")
                    self.abortMotion(gs)
                    was_aborted = True
                    break
                is_ready = (rv != fpu_driver.E_DriverErrCode.DE_WAIT_TIMEOUT)

        if was_aborted:
            self.pingFPUs(gs)
            print("findDatumw as aborted by SIGINT, movement stopped")
            raise MovementError("findDatum was aborted by SIGINT")

        return rv

    def pingFPUs(self, gs):
        return self._gd.pingFPUs(gs)

    def resetFPUs(self, gs):
        return self._gd.resetFPUs(gs)

    def getPositions(self, gs):
        return self._gd.getPositions(gs)

    def readRegister(self, address, gs):
        return self._gd.readRegister(address, gs)
    
    def getFirmwareVersion(self, gs):
        return self._gd.getFirmwareVersion(gs)
    
    def getCounterDeviation(self, gs):
        return self._gd.getCounterDeviation(gs)

    def configMotion(self, wavetable, gs, check_protection=True):
        """ 
        Configures movement by sending a waveform table to a group of FPUs.
        Call signature is configMotion({ fpuid0 : {(asteps,bsteps), (asteps, bsteps), ...], fpuid1 : { ... }, ...}})

        When the 'protected' flag is set to False, bypass all 
        hardware protection checks, which will allow to move a
        collided or uncalibrated FPU (even if the movement might damage
        the hardware).

        """
        return self._gd.configMotion(wavetable, gs, check_protection)

    def executeMotionB(self, gs):
        return self._gd.executeMotion(gs)
    

    def executeMotion(self, gs):
        # wait a short moment to avoid spurious collision.
        time.sleep(2.5)
        rv = self._gd.startExecuteMotion(gs)
        if rv != fpu_driver.E_DriverErrCode.DE_OK:
            print("rv=",rv)
            raise RuntimeError("FPUs not ready to move, driver error code = %r" % rv)
        time.sleep(0.1)
        time_interval = 0.1
        is_ready = False
        was_aborted = False
        refresh_state = False
        try:
            with SignalHandler() as sh:
                while not is_ready:
                    rv = self._gd.waitExecuteMotion(gs, time_interval)
                    if sh.interrupted:
                        print("STOPPING FPUs.")
                        self.abortMotion(gs)
                        was_aborted = True
                        break
                    is_ready = (rv != fpu_driver.E_DriverErrCode.DE_WAIT_TIMEOUT)
        except MovementError:
            refresh_state = True
            raise
            

        if (rv == fpu_driver.E_DriverErrCode.DE_OK) or was_aborted or refresh_state:
            # execute a ping to update positions
            # (this is only needed for protocol version 1)
            self.pingFPUs(gs)
                            
        if was_aborted:
            raise MovementError("executeMotion was aborted by SIGINT")
        
        return rv

    def abortMotion(self, gs):
        return self._gd.abortMotion(gs)

    def freeBetaCollision(self, fpu_id, direction,  gs):
        return self._gd.freeBetaCollision(fpu_id, direction, gs)
    
    def enableBetaCollisionProtection(self, gs):
        return self._gd.enableBetaCollisionProtection(gs)

    def reverseMotion(self, gs):
        return self._gd.reverseMotion(gs)

    def repeatMotion(self, gs):
        return self._gd.repeatMotion(gs)
