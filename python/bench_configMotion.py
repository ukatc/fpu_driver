from __future__ import print_function

import time
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

print("issuing findDatum:")
gd.findDatum(gs)

num_steps = 256

steps = [ (200, 100) for i in range(num_steps)]

wave_table = { fpu : steps for fpu in range(NUM_FPUS) }

print("issuing configMotion:")

t0 = time.time()
gd.configMotion(wave_table, gs)
t1 = time.time()

print("elapsed time: %f" % (t1 - t0))


                       






