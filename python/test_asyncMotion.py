import fpu_driver

from fpu_commands import *

NUM_FPUS = 10
gateway_adr_list = [ fpu_driver.GatewayAddress("127.0.0.1", p)
                     for p in [4700, 4701, 4702] ]


import time
import signal

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
    def __init__(self, nfpus):
        self._gd = fpu_driver.GridDriver(NUM_FPUS)

    def initializeDriver(self):
        return self._gd.initializeDriver()

    def connect(self, address_list):
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
                    print("WE NEED TO STOP THIS!!!")
                    break
                is_ready = (rv != fpu_driver.E_DriverErrCode.DE_COMMAND_TIMEOUT)
                
        return rv
    
            
            
        
    
gd = GridDriver(NUM_FPUS)

print("initializing driver: ", gd.initializeDriver())


print("connecting grid:", gd.connect(gateway_adr_list))


print("getting grid state:")
gs = gd.getGridState()

print("issuing findDatum:")
gd.findDatum(gs)


wt = { 0: [ ( 10, 20),
           ( 15, 21),
           ( 20, 22),
           ( 30, 25),
           ( 21, 22),
           ( 31, 25),
           ( 22, 22),
           ( 32, 25),
           ( 23, 22),
           ( 33, 25),
           ( 24, 22),
           ( 34, 25),
           ( 25, 22),
           ( 35, 25),
           ( 26, 22),
           ( 36, 25),
           ( 27, 22),
           ( 37, 25),
           ( 28, 22),
           ( 38, 25),
           ( 29, 22),
           ( 39, 25),
           ( 50, 19) ],
      
      1: [ (-11, 71),
           (-11, 75),
           (-11, 70),
           ( 20, 72),
           ( 30, 75),
           ( 21, 72),
           ( 31, 75),
           ( 22, 72),
           ( 32, 75),
           ( 23, 72),
           ( 34, 75),
           ( 25, 72),
           ( 35, 75),
           ( 26, 72),
           ( 36, 75),
           ( 27, 72),
           ( 37, 75),
           ( 28, 72),
           ( 38, 75),
           ( 29, 72),
           ( 39, 75),
           (-10, 78),
           ( -9, 75) ],
      
      2: [ ( 100, 91),
           (-100, 91),
           (   0, 92),
           (  20, 92),
           (  30, 95),
           (  21, 92),
           (  31, 95),
           (  22, 92),
           (  32, 95),
           (  23, 92),
           (  33, 95),
           (  24, -92),
           (  35, 95),
           (  26, -92),
           (  36, 95),
           (  27, 92),
           (  37, 95),
           (  28, 92),
           (  38, 95),
           (  29, 92),
           (  39, 95),
           (   0, 95),
           ( 100, 90) ],
}

gd.configMotion(wt, gs)

                       






