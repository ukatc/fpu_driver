import FpuGridDriver

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

print("configuring wavetable")
gd.configMotion(wt, gs)

print("issuing executeMotion!")
gd.executeMotion(gs)

                       






