import FpuGridDriver

from FpuGridDriver import TEST_GATEWAY_ADRESS_LIST

from fpu_commands import *

NUM_FPUS = 1


gd = FpuGridDriver.GridDriver(NUM_FPUS)

print("initializing driver: ", gd.initializeDriver())


print("connecting grid:", gd.connect(address_list=TEST_GATEWAY_ADRESS_LIST))


print("getting grid state:")
gs = gd.getGridState()

print("gs=", gs)

print("positions before:", list_positions(gs))

print("issuing pingFPUs and getting positions:")
gd.pingFPUs(gs)

print("positions:", list_positions(gs))


wt = { 0: [ ( 10, 20),
           ( 15, 21),
           ( 20, 22),
           ( 30, 25),
           ( 50, 19) ],
      
      1: [ (-11, 21),
           (-11, 25),
           (-11, 30),
           (-10, 28),
           ( -9, 25) ],
      
      2: [ ( 100, 21),
           (-100, 21),
           ( 0, 22),
           ( 0, 25),
           ( 100, 20) ],
}



