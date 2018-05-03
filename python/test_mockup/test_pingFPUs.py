from __future__ import print_function, division

import FpuGridDriver
from fpu_commands import *

NUM_FPUS = 10
gateway_adr_list = [ FpuGridDriver.GatewayAddress("127.0.0.1", p)
                     for p in [4700, 4701, 4702] ]

print("module version is:", FpuGridDriver.__version__, ", CAN PROTOCOL version:", FpuGridDriver.CAN_PROTOCOL_VERSION)

gd = FpuGridDriver.GridDriver(NUM_FPUS)

print("connecting grid:", gd.connect(gateway_adr_list))


print("getting grid state:")
gs = gd.getGridState()

print("positions before:", list_positions(gs))

print("issuing pingFPUs and getting positions:")
gd.pingFPUs(gs)

print("positions:", list_positions(gs))





