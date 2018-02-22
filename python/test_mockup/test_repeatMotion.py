import FpuGridDriver
from FpuGridDriver import REQD_ANTI_CLOCKWISE,  REQD_CLOCKWISE

from fpu_commands import *

NUM_FPUS = 10
gateway_adr_list = [ FpuGridDriver.GatewayAddress("127.0.0.1", p)
                     for p in [4700, 4701, 4702] ]


    
gd = FpuGridDriver.GridDriver(NUM_FPUS)

print("connecting grid:", gd.connect(gateway_adr_list))


print("getting grid state:")
gs = gd.getGridState()

print("issuing findDatum:")
gd.findDatum(gs)

wt = { 0: [ ( 125, 125),
            ( 125, 135),
            ( 130, 135),
            ( 125, 125),
            ( 115,  72),],
      
       1: [ ( 125, 125),
            ( 125, 135),
            ( 130, 135),
            ( 125, 125),
            (  35,  50), ],
      
       2: [ ( 125, 125),
            ( 125, 135),
            ( 130, 135),
            ( 125, 125),
            ( 100,  25), ],
}



print("configuring wavetable")
gd.configMotion(wt, gs)

gd.pingFPUs(gs)
print("start positions: ", list_positions(gs))

print("issuing executeMotion (1a)!")
gd.executeMotion(gs)

gd.pingFPUs(gs)
print("resulting interim out positions: ", list_positions(gs))


print("issuing repeatMotion!")
gd.repeatMotion(gs)
print("issuing executeMotion (1b)!")
gd.executeMotion(gs)

gd.pingFPUs(gs)
print("resulting outward positions: ", list_positions(gs))


print("issuing reverseMotion (2a)!")
gd.reverseMotion(gs)

print("issuing executeMotion (2b)!")
gd.executeMotion(gs)

gd.pingFPUs(gs)
print("resulting interim inward positions: ", list_positions(gs))


print("issuing reverseMotion (2a)!")
gd.reverseMotion(gs)

print("issuing executeMotion (2b)!")
gd.executeMotion(gs)

gd.pingFPUs(gs)
print("resulting return positions: ", list_positions(gs))

                       






