#!/usr/bin/python

import time
import signal

import fpu_driver

DEFAULT_GATEWAY_ADRESS_LIST = [ fpu_driver.GatewayAddress("127.0.0.1", p)
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

    def findDatum(self, gs):
        return self._gd.findDatum(gs)

    def pingFPUs(self, gs):
        return self._gd.pingFPUs(gs)

    def resetFPUs(self, gs):
        return self._gd.resetFPUs(gs)

    def getPositions(self, gs):
        return self._gd.getPositions(gs)

    def configMotion(self, wavetable, gs):
        return self._gd.configMotion(wavetable, gs)

    def executeMotion(self, gs):
        rv = self._gd.startExecuteMotion(gs)
        if rv != fpu_driver.E_DriverErrCode.DE_OK:
            raise rv
        print("wait a little..")
        time.sleep(0.5)
        time_interval = 0.25
        is_ready = False
        with SignalHandler() as sh:
            while not is_ready:
                print("python: waiting...")
                rv = self._gd.waitExecuteMotion(gs, time_interval)
                if sh.interrupted:
                    print("STOPPING FPUs!")
                    self.abortMotion(gs)
                    break
                is_ready = (rv != fpu_driver.E_DriverErrCode.DE_COMMAND_TIMEOUT)

        if rv == fpu_driver.E_DriverErrCode.DE_OK:
            # execute a ping to update positions
            # (this is only needed for protocol version 1)
            self.pingFPUs(gs)
            
        return rv

    def abortMotion(self, gs):
        return self._gd.abortMotion(gs)
