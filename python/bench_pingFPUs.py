from __future__ import print_function
import sys
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

print("positions before:", list_positions(gs))

print("issuing pingFPUs and getting positions:")
N = 1000
t0 = time.time()
for i in range(N):
    gd.pingFPUs(gs)
    if (i % 10) == 0 :
        print("\r %06i" % i, end='')
        sys.stdout.flush()

t1 = time.time()

print("\npositions:", list_positions(gs))

msgs_per_sec = (NUM_FPUS * N * 2.0) / (t1 - t0)
print("bench result: %f messages/sec" % msgs_per_sec)




