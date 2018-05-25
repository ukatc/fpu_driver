from __future__ import print_function

import FpuGridDriver
from FpuGridDriver import TEST_GATEWAY_ADRESS_LIST, \
    SEARCH_CLOCKWISE, SEARCH_ANTI_CLOCKWISE, SEARCH_AUTO, SKIP_FPU

from fpu_commands import *

NUM_FPUS = 1


gd = FpuGridDriver.GridDriver(NUM_FPUS)

print("connecting grid:", gd.connect(address_list=TEST_GATEWAY_ADRESS_LIST))


print("getting grid state:")
gs = gd.getGridState()

print("gs=", gs)

print("positions before:", list_positions(gs))

print("issuing pingFPUs and getting positions:")
gd.pingFPUs(gs)

print("positions:", list_positions(gs))


wt = { 0: [ ( 125, 125),
           ( 130, 130),
           ( 135, 130),
           ( 125, 125),
           ( 50, 40) ],
      
}



