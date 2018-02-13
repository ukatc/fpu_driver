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



print("issuing findDatum:")
gd.findDatum(gs)
print("findDatum finished")



steps_up = [ (100, 0) for i in range(125) ]

steps_down = [ (-100, 0) for i in range(125) ]

wt_up = { 0: steps_up, }

wt_down = { 0: steps_down, }


print("issuing configMotion:")

rv = gd.configMotion(wt_up, gs)

print("configMotion finished with return value", rv)

print("issuing executeMotion - press <Ctrl-C> to test abortMotion message.")

#gd.executeMotion(gs)



                       






