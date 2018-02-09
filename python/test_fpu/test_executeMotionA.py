from __future__ import print_function
import time

import FpuGridDriver

from FpuGridDriver import TEST_GATEWAY_ADRESS_LIST

from fpu_commands import *

NUM_FPUS = 1


gd = FpuGridDriver.GridDriver(NUM_FPUS)

print("initializing driver: ", gd.initializeDriver())


print("connecting grid:", gd.connect(address_list=TEST_GATEWAY_ADRESS_LIST))


print("getting grid state:")
gs = gd.getGridState()

if 0:
    # does not seem to work always
    print("resetting FPU..")
    rv = gd.resetFPUs(gs)
    print("rv=", rv)
    print("waiting 5 seconds...")
    time.sleep(5)


print("issuing findDatum:")
gd.findDatum(gs)
print("findDatum finished")


wt_ab = { 0: [ ( 100, 100),
            ( 100, 100),
            ( 100, 100),
            ( 110, 100),
            ( 105, 100),
            ( 100, 100),
            ( 100, 100) ],      
}

wt = { 0: [ ( -10, 10),
 ],      
}
print("issuing configMotion:")

rv = gd.configMotion(wt, gs)

print("configMotion finished with return value", rv)


                       






