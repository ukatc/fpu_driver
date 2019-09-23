from __future__ import print_function
import FpuGridDriver
from FpuGridDriver import REQD_ANTI_CLOCKWISE,  REQD_CLOCKWISE

from fpu_commands import *

NUM_FPUS = 10
gateway_adr_list = [ FpuGridDriver.GatewayAddress("127.0.0.1", p)
                     for p in [4700, 4701, 4702] ]



gd = FpuGridDriver.GridDriver(NUM_FPUS, mockup=True)

print("connecting grid:", gd.connect(gateway_adr_list))


print("getting grid state:")
gs = gd.getGridState()

print("issuing findDatum:")
gd.findDatum(gs)


wt = { 0: [ ( 125, 125),
            ( 125, 135),
            ( 130, 135),
            ( 125, 125),
            ( 120,  72),],

       1: [ ( 125, 125),
            ( 125, 135),
            ( 130, 135),
            ( 125, 125),
            ( 115, 100), ],

       2: [ ( 125, 125),
            ( 125, 135),
            ( 130, 135),
            ( 125, 130),
            ( 100, 125), ],

}
gd.configMotion(wt, gs)
