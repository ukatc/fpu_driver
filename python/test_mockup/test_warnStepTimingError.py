from __future__ import print_function

from FpuGridDriver import GridDriver
from FpuGridDriver import REQD_ANTI_CLOCKWISE,  REQD_CLOCKWISE


from fpu_commands import *

NUM_FPUS = 10
USTEP_LEVEL = 8

gd = GridDriver(NUM_FPUS, mockup=True)

print("connecting grid:", gd.connect())


print("getting grid state:")
gs = gd.getGridState()

gd.pingFPUs(gs)
gd.setUStepLevel(USTEP_LEVEL, gs)



print("issuing findDatum:")
gd.findDatum(gs)


wt = { 0: [ ( 125, 125),
            ( 125, 135),
            ( 130, 135),
            ( 140, 145),
            ( 145, 142),
            ( 141, 145),
            ( 135, 135),
            ( 132, 135),
            ( 125, 135),
            ( 133, 125),
            ( 125, 125),
            ( 134, 125),
            ( 125, 125),
            ( 135, 125),
            ( 126, 125),
            ( 136, 125),
            ( 127, 125),
            ( 137, 125),
            ( 138, 125),
            ( 148, 125),
            ( 139, 125),
            ( 125, 125),
            (  50, 19) ],

       1: [ ( 125, 125),
            ( 125, 135),
            ( 130, 135),
            ( 140, 145),
            ( 145, 142),
            ( 141, 145),
            ( 155, 135),
            ( 152, 135),
            ( 153, 135),
            ( 163, 125),
            ( 164, 125),
            ( 164, 125),
            ( 175, 125),
            ( 155, 125),
            ( 156, 125),
            ( 146, 125),
            ( 137, 125),
            ( 125, 125),
            ( 128, 125),
            ( 128, 125),
            ( 129, 125),
            ( 125, 125),
            (  50, 19) ],

       2: [ ( 125, 125),
            ( 125, 135),
            ( 130, 135),
            ( 140, 145),
            ( 145, 142),
            ( 141, 145),
            ( 135, 135),
            ( 132, 135),
            ( 125, 135),
            ( 143, 125),
            ( 144, 125),
            ( 154, 125),
            ( 155, 125),
            ( 165, 125),
            ( 166, 125),
            ( 176, 125),
            ( 167, 125),
            ( 157, 125),
            ( 148, 125),
            ( 138, 125),
            ( 139, 125),
            ( 125, 125),
            (  50, 19) ],

}

gd.configMotion(wt, gs)

gd.executeMotion(gs)
