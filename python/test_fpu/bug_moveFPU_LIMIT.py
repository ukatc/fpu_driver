from __future__ import print_function
import time


import FpuGridDriver
from FpuGridDriver import REQD_ANTI_CLOCKWISE,  REQD_CLOCKWISE
from FpuGridDriver import TEST_GATEWAY_ADRESS_LIST

from fpu_commands import *

NUM_FPUS = 1




gd = FpuGridDriver.GridDriver(NUM_FPUS)

print("connecting grid:", gd.connect(address_list=TEST_GATEWAY_ADRESS_LIST))


# We monitor the FPU grid by a variable which is
# called grid_state, and reflects the state of
# all FPUs.
print("getting grid state:")
grid_state = gd.getGridState()


# Now, we issue a findDatum method. In order to know when how this
# command finished, we pass the grid_state variable.
print("issuing findDatum:")
gd.findDatum(grid_state)
print("findDatum finished")

# We can use grid_state to display the starting position
print("the starting position (in degrees) is:", list_angles(grid_state))



# Now, let's generate a waveform which moves the alpha arm by + 90
# degree, and the beta arm by +45 degree. Positive angles always mean
# "counterclockwise" (when viewed from above).


for alpha, beta in [ (0, 0), (1, 0), (2, 0), (5, 0), (7.5, 0), (10, 0),
                     (12.5, 0),
                     (15, 0), (30, 0),
                     (45, 0), (60, 0), (90, 0),
                     (135, 0), (150, 0), (175, 0), (180, 0),
                     (210, 0), (225, 0), (240, 0), (270, 0), (285, 0),
                     (300, 0), (315,0), (330, 0), (345, 0),
                     
                     (0, 1), (0, 2), (0, 5), (0, 7.5), (0, 10),
                     (0, 15), (0, 30), (0, 45), (0, 60), (0, 90),
                     (0, 135), (0, 150), (0, 175), (0, 180),
                     
                     (0, -1), (0, -2), (0, -5), (0, -7.5), (0, -10),
                     (0, -15), (0, -30), (0, -45), (0, -60), (0, -90),
                     (0, -135), (0, -150), (0, -175), (0, -180),
                     ]:
    

    for sign in [1, -1]:
        alpha_move = sign * alpha
        beta_move = sign * beta
        # the following function generates a waveform for one FPU
        waveform = gen_wf(alpha_move, beta_move)

        # configure this movement waveform, by
        # uploading it to the FPU
        gd.configMotion(waveform, grid_state)

        # wait a moment
        time.sleep(2)
        
        # start the movement!
        print("starting movement by"
              +" ({:5.1f},{:5.1f}) degree".format(alpha_move, beta_move))
        gd.executeMotion(grid_state)

        # display the new position, using our grid_state
        # information
        print("the new position (in degrees) is:", list_angles(grid_state))
                  
        # we can print the motor steps as well
        print("in motor step units, the new position is:", list_positions(grid_state))

        # wait a moment 
        print("we wait 1 sec..")
        time.sleep(1)


print("issuing findDatum:")
gd.findDatum(grid_state)
        
gd.getCounterDeviation(grid_state)

# we get the state for FPU 0 (the only one)
fpu_state = grid_state.FPU[0]
print("the counter deviation for FPU 0 is "
      "(dev_alpha, dev_beta) = ({}, {}) steps".format(
          fpu_state.alpha_deviation,
          fpu_state.beta_deviation))
        
print("ready.")
                  

      









