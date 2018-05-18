from __future__ import print_function

import FpuGridDriver
from FpuGridDriver import TEST_GATEWAY_ADRESS_LIST
from FpuGridDriver import REQD_ANTI_CLOCKWISE,  REQD_CLOCKWISE, \
    DASEL_BOTH, DASEL_ALPHA, DASEL_BETA

from fpu_commands import *

NUM_FPUS = 1


gd = FpuGridDriver.GridDriver(NUM_FPUS)


print("connecting grid:", gd.connect(address_list=TEST_GATEWAY_ADRESS_LIST))


print("getting grid state:")
gs = gd.getGridState()


print("getting positions:")
gd.getPositions(gs)

print("positions before:", list_positions(gs))

print("finding Datum")
gd.findDatum(gs, DASEL_BETA)
 
print("set positions after:", list_positions(gs))

print("getting positions:")
gd.getPositions(gs)

print("reported positions after:", list_positions(gs))






