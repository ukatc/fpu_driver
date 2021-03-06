from __future__ import print_function, division

import FpuGridDriver
from FpuGridDriver import  DASEL_BOTH, DASEL_ALPHA, DASEL_BETA, \
    SEARCH_CLOCKWISE, SEARCH_ANTI_CLOCKWISE, SEARCH_AUTO, SKIP_FPU

from fpu_commands import *

NUM_FPUS = 10
gateway_adr_list = [ FpuGridDriver.GatewayAddress("127.0.0.1", p)
                     for p in [4700, 4701, 4702] ]

print("module version is:", FpuGridDriver.__version__, ", CAN PROTOCOL version:", FpuGridDriver.CAN_PROTOCOL_VERSION)

gd = FpuGridDriver.GridDriver(NUM_FPUS, mockup=True)

print("connecting grid:", gd.connect(gateway_adr_list))


print("getting grid state:")
gs = gd.getGridState()

print("positions before:", list_positions(gs))

print("issuing pingFPUs and getting positions:")
gd.pingFPUs(gs)

print("positions:", list_positions(gs))

print("Serial numbers:")
gd.printSerialNumbers(gs)
