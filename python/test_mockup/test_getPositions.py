import fpu_driver
from fpu_commands import *

NUM_FPUS = 1000
gateway_adr_list = [ fpu_driver.GatewayAddress("127.0.0.1", p)
                     for p in [4700, 4701, 4702] ]


gd = fpu_driver.GridDriver(NUM_FPUS)

print("initializing driver: ", gd.initializeDriver())


print("connecting grid:", gd.connect(gateway_adr_list))


print("getting grid state:")
gs = gd.getGridState()

print("positions before:", list_positions(gs))

print("getting positions:")
gd.getPositions(gs)

print("positions:", list_positions(gs))




