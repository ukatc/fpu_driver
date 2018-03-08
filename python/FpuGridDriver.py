#!/usr/bin/python

import time
import signal

import fpu_driver

from fpu_driver import __version__, CAN_PROTOCOL_VERSION, GatewayAddress,  \
    REQD_ANTI_CLOCKWISE,  REQD_CLOCKWISE, FPUDriverException

import fpu_commands as cmds


DEFAULT_GATEWAY_ADRESS_LIST = [ GatewayAddress("127.0.0.1", p)
                                for p in [4700, 4701, 4702] ]

TEST_GATEWAY_ADRESS_LIST = [ GatewayAddress("192.168.0.10", 4700) ]


DEFAULT_NUM_FPUS=1005


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

class GridDriver:
    def __init__(self, nfpus=DEFAULT_NUM_FPUS):
        self._gd = fpu_driver.GridDriver(nfpus)

    def connect(self, address_list=DEFAULT_GATEWAY_ADRESS_LIST):
        return self._gd.connect(address_list)

    def setUStepLevel(self, ustep_level,  gs):
        return self._gd.setUStepLevel(ustep_level, gs)
    
    def getGridState(self):
        return self._gd.getGridState()

    def findDatumB(self, gs):
        """Moves all FPUs to datum position. 

        This is a blocking variand of the findDatum command,
        it is not interruptible by Control-C."""
        
        return self._gd.findDatum(gs)

    def findDatum(self, gs):
        """Moves all FPUs to datum position. 

        If the program receives a SIGNINT, or Control-C is pressed, an
        abortMotion command is sent, aborting the search.

        """
        rv = self._gd.startFindDatum(gs)
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
                is_ready = (rv != fpu_driver.E_DriverErrCode.DE_COMMAND_TIMEOUT)

        if was_aborted:
            self.pingFPUs(gs)
            print("findDatumw as aborted by SIGINT, movement stopped")
            raise RuntimeError("findDatum was aborted by SIGINT")

        return rv

    def pingFPUs(self, gs):
        return self._gd.pingFPUs(gs)

    def resetFPUs(self, gs):
        return self._gd.resetFPUs(gs)

    def getPositions(self, gs):
        return self._gd.getPositions(gs)

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
                    is_ready = (rv != fpu_driver.E_DriverErrCode.DE_COMMAND_TIMEOUT)
        except FpuDriverException as rtex:
            errtype = str(rtex).split(":")[0].strip()
            if errtype in [ "DE_STEP_TIMING_ERROR", "DE_NEW_COLLISION", "DE_NEW_LIMIT_BREACH"] :
                refresh_state = True
            raise
            

        if (rv == fpu_driver.E_DriverErrCode.DE_OK) or was_aborted or refresh_state:
            # execute a ping to update positions
            # (this is only needed for protocol version 1)
            self.pingFPUs(gs)
                            
        if was_aborted:
            raise RuntimeError("executeMotion was aborted by SIGINT")
        
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
