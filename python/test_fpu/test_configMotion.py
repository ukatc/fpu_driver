from __future__ import print_function

import FpuGridDriver

from FpuGridDriver import *

from fpu_commands import *

NUM_FPUS = 1


gd = FpuGridDriver.GridDriver(NUM_FPUS)

print("connecting grid:", gd.connect(address_list=TEST_GATEWAY_ADRESS_LIST))


print("getting grid state:")
gs = gd.getGridState()

print("issuing findDatum:")
gd.findDatum(gs)
print("findDatum finished")


wt = { 0: [ ( 125,0),
            ( 130, 0),
            ( 140, 0),
            ( 150, 0),
            ( 135, 0),
            ( 125, 0),
            ( 50, 0) ],
      
}
print("issuing configMotion:")

rv = gd.configMotion(wt, gs)

print("configMotion finished with return value", rv)


                       






