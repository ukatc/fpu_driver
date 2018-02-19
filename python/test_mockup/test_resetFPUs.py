import FpuGridDriver
from fpu_commands import *

NUM_FPUS = 100
gateway_adr_list = [ FpuGridDriver.GatewayAddress("127.0.0.1", p)
                     for p in [4700, 4701, 4702] ]

    
gd = FpuGridDriver.GridDriver(NUM_FPUS)

print("connecting grid:", gd.connect(gateway_adr_list))


print("getting grid state:")
gs = gd.getGridState()

print("positions before:", list_positions(gs))

print("issuing pingFPUs and getting positions:")
gd.resetFPUs(gs)






