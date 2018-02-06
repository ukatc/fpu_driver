#!/usr/bin/python

import time
import signal

import fpu_driver
from fpu_driver import GatewayAddress
import fpu_commands as cmds


DEFAULT_GATEWAY_ADRESS_LIST = [ GatewayAddress("127.0.0.1", p)
                                for p in [4700, 4701, 4702] ]


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

    def initializeDriver(self):
        return self._gd.initializeDriver()

    def connect(self, address_list=DEFAULT_GATEWAY_ADRESS_LIST):
        return self._gd.connect(address_list)

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
        print("wait a little..")
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
        else:
            print("findDatum finished.")
        return rv

    def pingFPUs(self, gs):
        return self._gd.pingFPUs(gs)

    def resetFPUs(self, gs):
        return self._gd.resetFPUs(gs)

    def getPositions(self, gs):
        return self._gd.getPositions(gs)

    def getCounterDeviation(self, gs):
        return self._gd.getCounterDeviation(gs)

    def configMotion(self, wavetable, gs):
        """ 
        Configures movement by sending a waveform table to a group of FPUs.
        Call signature is configMotion({ fpuid0 : {(asteps,bsteps), (asteps, bsteps), ...], fpuid1 : { ... }, ...}})

        """
        return self._gd.configMotion(wavetable, gs)

    def executeMotionB(self, gs):
        return self._gd.executeMotion(gs)
    

    def executeMotion(self, gs):
        rv = self._gd.startExecuteMotion(gs)
        if rv != fpu_driver.E_DriverErrCode.DE_OK:
            print("rv=",rv)
            raise RuntimeError("FPUs not ready to move, driver error code = %r" % rv)
        print("wait a little..")
        time.sleep(0.1)
        time_interval = 0.1
        is_ready = False
        was_aborted = False
        with SignalHandler() as sh:
            while not is_ready:
                rv = self._gd.waitExecuteMotion(gs, time_interval)
                if sh.interrupted:
                    print("STOPPING FPUs.")
                    self.abortMotion(gs)
                    was_aborted = True
                    break
                is_ready = (rv != fpu_driver.E_DriverErrCode.DE_COMMAND_TIMEOUT)

        if rv == fpu_driver.E_DriverErrCode.DE_OK:
            # execute a ping to update positions
            # (this is only needed for protocol version 1)
            self.pingFPUs(gs)

        if not was_aborted:
            print("executeMotion finished")
        else:
            self.pingFPUs(gs)
            raise RuntimeError("executeMotion was aborted by SIGINT")
        
        return rv

    def abortMotion(self, gs):
        return self._gd.abortMotion(gs)

    def reverseMotion(self, gs):
        return self._gd.reverseMotion(gs)

    def repeatMotion(self, gs):
        return self._gd.repeatMotion(gs)
