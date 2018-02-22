from __future__ import print_function

import time
import FpuGridDriver
from FpuGridDriver import REQD_ANTI_CLOCKWISE,  REQD_CLOCKWISE

from fpu_commands import *

NUM_FPUS = 1000

gateway_adr_list = [ FpuGridDriver.GatewayAddress("127.0.0.1", p)
                     for p in [4700, 4701, 4702] ]


    
gd = FpuGridDriver.GridDriver(NUM_FPUS)

print("connecting grid:", gd.connect(gateway_adr_list))


print("getting grid state:")
gs = gd.getGridState()

print("issuing findDatum:")
gd.findDatum(gs)

num_sections = 128

steps = [ (125, 125) for i in range(num_sections)]

wave_table = { fpu : steps for fpu in range(NUM_FPUS) }

print("issuing configMotion:")

t0 = time.time()
gd.configMotion(wave_table, gs)
t1 = time.time()

print("elapsed time: %f" % (t1 - t0))


                       






