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

print("resetting FPUs:")
gd.resetFPUs(gs)

print("issuing findDatum:")
gd.findDatum(gs)


wt = { 0: [ ( 10, 20),
           ( 15, 21),
           ( 20, 22),
           ( 30, 25),
           ( 30, 25),
           ( 50, 19) ],
      
      1: [ (-11, 5021),
           (-11, 5025),
           (-11, 5030),
           (-11, 5030),
           (-10, 5028),
           ( -9, 5025) ],
      
      2: [ ( 20, 21),
           ( 20, 21),
           ( 20, 22),
           ( 20, 25),
           ( 20, 25),
           ( 10, 20) ],
}

print("configuring wavetable")
gd.configMotion(wt, gs)

gd.reverseMotion(gs)
print("issuing executeMotion! (this should fail)")
try:
    gd.executeMotion(gs)
except RuntimeError:
    print("exception caught..")

print("trying to free collision...")
gd.freeBetaCollision(1, REQD_ANTI_CLOCKWISE, gs)
gd.freeBetaCollision(1, REQD_ANTI_CLOCKWISE, gs)

gd.pingFPUs(gs)
print("new positions:", list_positions(gs))
print("enable protection...")
gd.enableBetaCollisionProtection(gs)

print("configuring wavetable again (old data is invalid)")
gd.configMotion(wt, gs)
print("issuing executeMotion! (this should work)")

gd.executeMotion(gs)



                       






