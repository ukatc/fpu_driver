import FpuGridDriver
from fpu_commands import *

NUM_FPUS = 1000
gateway_adr_list = [ FpuGridDriver.GatewayAddress("127.0.0.1", p)
                     for p in [4700, 4701, 4702] ]


gd = FpuGridDriver.GridDriver(NUM_FPUS)

print("initializing driver: ", gd.initializeDriver())


print("connecting grid:", gd.connect(gateway_adr_list))


print("getting grid state:")
gs = gd.getGridState()


print("getting positions:")
gd.getPositions(gs)

print("positions before:", list_positions(gs))

print("finding Datum")
gd.findDatum(gs)

print("set positions after:", list_positions(gs))

print("getting counter deviations:")
gd.getCounterDeviation(gs)

print("reported deviations:", list_deviations(gs))








