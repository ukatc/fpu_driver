from __future__ import print_function
import time


import FpuGridDriver

from FpuGridDriver import TEST_GATEWAY_ADRESS_LIST

from fpu_commands import *

NUM_FPUS = 1




gd = FpuGridDriver.GridDriver(NUM_FPUS)

print("initializing driver: ", gd.initializeDriver())


print("connecting grid:", gd.connect(address_list=TEST_GATEWAY_ADRESS_LIST))


print("getting grid state:")
gs = gd.getGridState()



print("issuing findDatum:")
gd.findDatum(gs)
print("findDatum finished")



alpha_move = 45
beta_move = 30
wf = gen_wf(alpha_move, beta_move)

gd.configMotion(wf, gs)






