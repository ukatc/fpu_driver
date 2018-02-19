from __future__ import print_function
import time

import FpuGridDriver

from FpuGridDriver import TEST_GATEWAY_ADRESS_LIST

from fpu_commands import *

NUM_FPUS = 1


gd = FpuGridDriver.GridDriver(NUM_FPUS)

print("connecting grid:", gd.connect(address_list=TEST_GATEWAY_ADRESS_LIST))


print("getting grid state:")
gs = gd.getGridState()



print("issuing findDatum:")
gd.findDatum(gs)
print("findDatum finished")



steps_up = [ (100, 0) for i in range(125) ]

steps_down = [ (-100, 0) for i in range(125) ]

wt_up = { 0: steps_up, }

wt_down = { 0: steps_down, }


print("issuing configMotion:")

rv = gd.configMotion(wt_down, gs)

print("configMotion finished with return value", rv)


print("issuing 1st executeMotion")
gd.executeMotion(gs)
print("positions:", list_positions(gs))

# gd.repeatMotion(gs)
# print("issuing 2nd executeMotion")
# gd.executeMotion(gs)
# print("positions:", list_positions(gs))
# 
# gd.repeatMotion(gs)
# print("issuing 3rd executeMotion")
# gd.executeMotion(gs)
# print("positions:", list_positions(gs))
# 
# 
# gd.reverseMotion(gs)
# print("issuing 4th executeMotion")
# gd.executeMotion(gs)
# print("positions:", list_positions(gs))
# 
# gd.reverseMotion(gs)
# print("issuing 5th executeMotion")
# gd.executeMotion(gs)
# print("positions:", list_positions(gs))
# 
# gd.reverseMotion(gs)
# print("issuing 6th executeMotion")
# gd.executeMotion(gs)
# print("positions (should be back to origin):", list_positions(gs))
# 
# 
# 
# 
# 
# 
#                        






