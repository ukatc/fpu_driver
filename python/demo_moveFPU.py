from __future__ import print_function
import time


import FpuGridDriver

from FpuGridDriver import TEST_GATEWAY_ADRESS_LIST

from fpu_commands import *

NUM_FPUS = 1

if os.environ.get("MOCKUP","") != "":
    mockup = True
else:
    mockup = False



gd = FpuGridDriver.GridDriver(NUM_FPUS, mockup=mockup)

print("Connecting grid:", gd.connect(address_list=TEST_GATEWAY_ADRESS_LIST))


# We monitor the FPU grid by a variable which is
# called grid_state, and reflects the state of
# all FPUs.
print("getting Grid state:")
grid_state = gd.getGridState()


# Now, we issue a findDatum method. In order to know when how this
# command finished, we pass the grid_state variable.
print("issuing findDatum:")
gd.findDatum(grid_state)
print("findDatum finished")

# We can use grid_state to display the starting position
print("The starting position (in degrees) is:", list_angles(grid_state))



# Now, let's generate a waveform which moves the alpha arm by + 90
# degree, and the beta arm by +45 degree. Positive angles always mean
# "counterclockwise" (when viewed from above).

alpha_move = 45
beta_move = 15

# the following function generates a waveform for one FPU
waveform = gen_wf(alpha_move, beta_move)

# configure this movement waveform, by
# uploading it to the FPU
gd.configMotion(waveform, grid_state)

# start the movement!
print("starting movement by (45,15) degree")
gd.executeMotion(grid_state)

# display the new position, using our grid_state
# information
print("the new position (in degrees) is:", list_angles(grid_state))

# we can print the motor steps as well
print("in motor step units, the new position is:", list_positions(grid_state))

# wait a moment to enjoy that great success
print("we wait three seconds..")
time.sleep(3)

# configure to repeat the movement
gd.repeatMotion(grid_state)
# start the movement!
print("starting another movement by (45,15) degree")
gd.executeMotion(grid_state)

# display the new position
print("the second position (in degrees) is:", list_angles(grid_state))

print("in motor step units, the new position is:", list_positions(grid_state))

# wait a moment
print("we wait two seconds..")
time.sleep(2)


# now, we move from (90, 30) to (60, 0) degree
alpha_move = -30
beta_move = -30

waveform2 = gen_wf(alpha_move, beta_move)

# configure this waveform
gd.configMotion(waveform2, grid_state)

# start the movement!
print("starting movement by (-30,-30) degree to (60, 0)")
gd.executeMotion(grid_state)
print("the third position (in degrees) is:", list_angles(grid_state))

print("reversing movement by (+30,+30) degree to (90, 30)")

gd.reverseMotion(grid_state)
gd.executeMotion(grid_state)

# display the new position
print("the fourth, reversed position (in degrees) is now:",
      list_angles(grid_state))

# wait a short moment
print("we wait one second..")
time.sleep(1)


# now, we issue findDatum again, and look for the counter deviation
print("issuing findDatum:")
gd.findDatum(grid_state)

gd.getCounterDeviation(grid_state)

# we get the state for FPU 0 (the only one)
fpu_state = grid_state.FPU[0]
print("the counter deviation for FPU 0 is "
      "(dev_alpha, dev_beta) = ({}, {}) steps".format(
          fpu_state.alpha_deviation,
          fpu_state.beta_deviation))
