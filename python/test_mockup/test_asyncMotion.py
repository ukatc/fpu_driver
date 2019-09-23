from __future__ import print_function
from FpuGridDriver import GridDriver
from FpuGridDriver import REQD_ANTI_CLOCKWISE,  REQD_CLOCKWISE


from fpu_commands import *

NUM_FPUS = 10
USTEP_LEVEL = 4

gd = GridDriver(NUM_FPUS, mockup=True)

print("connecting grid:", gd.connect())


print("getting grid state:")
gs = gd.getGridState()

gd.pingFPUs(gs)



print("issuing findDatum:")
gd.findDatum(gs)


wt = { 0: [ ( 62, 62),
            ( 62, 67),
            ( 65, 67),
            ( 70, 73),
            ( 73, 71),
            ( 70, 73),
            ( 67, 67),
            ( 66, 67),
            ( 62, 67),
            ( 66, 62),
            ( 62, 62),
            ( 67, 62),
            ( 62, 62),
            ( 67, 62),
            ( 63, 62),
            ( 68, 62),
            ( 63, 62),
            ( 68, 62),
            ( 68, 62),
            ( 74, 62),
            ( 70, 62),
            ( 62, 62),
            (  25, 19) ],

       1: [ ( 62, 62),
            ( 62, 67),
            ( 65, 67),
            ( 70, 73),
            ( 73, 71),
            ( 70, 73),
            ( 72, 67),
            ( 76, 67),
            ( 76, 67),
            ( 82, 62),
            ( 82, 62),
            ( 82, 62),
            ( 87, 62),
            ( 72, 62),
            ( 78, 62),
            ( 73, 62),
            ( 68, 62),
            ( 62, 62),
            ( 64, 62),
            ( 64, 62),
            ( 64, 62),
            ( 62, 62),
            (  25, 19) ],

       2: [ ( 62, 62),
            ( 62, 67),
            ( 65, 67),
            ( 70, 73),
            ( 73, 71),
            ( 70, 73),
            ( 67, 67),
            ( 66, 67),
            ( 62, 67),
            ( 72, 62),
            ( 72, 62),
            ( 76, 62),
            ( 72, 62),
            ( 82, 62),
            ( 83, 62),
            ( 85, 62),
            ( 83, 62),
            ( 77, 62),
            ( 74, 62),
            ( 68, 62),
            ( 70, 62),
            ( 62, 62),
            (  25, 19) ],

}

gd.configMotion(wt, gs)

gd.executeMotion(gs)
