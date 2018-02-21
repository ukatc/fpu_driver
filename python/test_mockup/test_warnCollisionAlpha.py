import FpuGridDriver

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

wt = gen_wf(40, 0)

print("configuring wavetable")
gd.configMotion(wt, gs)

gd.reverseMotion(gs)
print("issuing executeMotion!")
gd.executeMotion(gs)

                       






