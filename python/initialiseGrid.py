#!/usr/bin/env python

"""

This Python script provides an interactive interface where an expert user can
communicate with the FPU grid driver software. The script initialises the
grid by connecting to the EtherCAN interface, creating a grid state object
and pinging the FPUs, then it hands over to the user.

The script can be started by the command

    python -i initialiseGrid.py -N <number of FPUs> --gateway_address <IP address>

For example (for 3 FPUs)

    python -i initialiseGrid.py -N 3 --gateway_address 192.168.0.10
    python -i initialiseGrid.py -N 3 --gateway_address 192.168.0.11

If EtherCAN hardware is not available, the script can communicate with the EtherCAN
simulator (mock gateway) with the command:

    python -i initialiseGrid.py -N <number of FPUs> --mockup

The mock gateway must be started first.

One the script has started, the user can enter commands in response to the Python prompt.
For example, if the fibre positioners are shown near their datum positions, a datum
command can be issued:

>>> gd.findDatum( gs )

If the positioners are not near their datum positions, they can be datumed with the
sequence

>>> gd.configDatum( gs )
>>> gd.executeMotion( gs )
>>> gd.findDatum( gs )

The following command will display a tabular summary of the current state of the grid

>>> check_status( gs )

The following command can be used to execute and reverse a path

>>> test_path( gd, gs, path_file, canmap_file, fpuset=[] )

The following command can be used to recover from a fault or collision

>>> recover_faults( gd, gs, fpuset=[], move_alpha=True, move_beta=True, distance=1.0, verbose=False)

The following command can be used to dump the current FPU locations to a target
status file which can be given to the path analysis software.

>>> save_state_to_file( gd, gs, status_file, config_file, canmap_file, fpuset=[] )

"""

from __future__ import print_function, division


import os
import argparse

import FpuGridDriver
from FpuGridDriver import  DASEL_BOTH, DASEL_ALPHA, DASEL_BETA, \
    SEARCH_CLOCKWISE, SEARCH_ANTI_CLOCKWISE, SEARCH_AUTO, SKIP_FPU, \
    REQD_CLOCKWISE, REQD_ANTI_CLOCKWISE, REQD_NEGATIVE, REQD_POSITIVE, \
    DATUM_TIMEOUT_DISABLE, DIRST_CLOCKWISE, DIRST_ANTI_CLOCKWISE, \
    DIRST_RESTING_LAST_CW, DIRST_RESTING_LAST_ACW

try:
   import wflib
except:
   wflib = None

from fpu_constants import RADIAN_TO_DEGREE
from fpu_commands import *

NUM_FPUS = int(os.environ.get("NUM_FPUS","10"))

def show_help():
    print(__doc__)

def parse_args():
    global help_text
    parser = argparse.ArgumentParser(description="""
Configure the MOONS FPU device driver to communicate with a grid of FPUs,
then ping the FPUs. Use python -i initialiseGrid.py for interactive mode.
    """)
    parser.add_argument('--mockup',   default=False, action='store_true',
                        help='set gateway address to use mock-up gateway and FPU')

    parser.add_argument('--resetFPU',   default=False, action='store_true',
                        help='reset FPU so that earlier aborts / collisions are ignored')

    parser.add_argument('--gateway_port', metavar='GATEWAY_PORT', type=int, default=4700,
                        help='EtherCAN gateway port number (default: %(default)s)')

    parser.add_argument('--gateway_address', metavar='GATEWAY_ADDRESS', type=str, default="192.168.0.10",
                        help='EtherCAN gateway IP address or hostname (default: %(default)r)')

    parser.add_argument('-N', '--NUM_FPUS',  metavar='NUM_FPUS', dest='N', type=int, default=NUM_FPUS,
                        help='Number of adressed FPUs (default: %(default)s).')


    args = parser.parse_args()
    return args


def initialize_FPU(args):
    # Execute the common sequence of commands to initialize the FPU grid.
    gd = FpuGridDriver.GridDriver(args.N, mockup=args.mockup)

    if args.mockup:
        gateway_address = [ FpuGridDriver.GatewayAddress("127.0.0.1", p)
                            for p in [4700, 4701, 4702] ]
    else:
        gateway_address = [ FpuGridDriver.GatewayAddress(args.gateway_address, args.gateway_port) ]

    try:
       print("Connecting grid:", gd.connect(address_list=gateway_address))
    except Exception as e:
       strg = "Could not connect to the gateways:\n\t%s\n" % str(e)
       if args.mockup:
           strg += "Have you started the mock gateway process? "
           strg += "Is the mock gateway simulating enough FPUs?"
       else:
           strg += "Are the gateways powered on? "
           strg += "Is there a network connection to the gateways?\n"
           strg += "Are there enough FPUs connected to the gateways? "
           strg += "Are the gateways being used by someone else?"
       raise Exception(strg)


    # We monitor the FPU grid by a variable which is
    # called grid_state, and reflects the state of
    # all FPUs.
    grid_state = gd.getGridState()

    if args.resetFPU:
        print("Resetting FPU")
        gd.resetFPUs(grid_state)
        print("OK")

    return gd, grid_state


def move_to( fpu, alpha_deg, beta_deg, calibrated=True ):
    strg = """
	*** NOTE: Calibrated movements can only be made from the verification software.
	*** Datum the fibre positioners and change to the vfrig Python shell.

	>>> gd.configDatum(gs)
	>>> gd.executeMotion(gs)
	>>> gd.findDatum(gs)
	>>> exit

	$ cd ~/FPUVerification
	$ python -i vfrigInitialiseGrid -N <number of FPUs> --gateway_address <IP address>
	>>> move_to( fpu, alpha_deg, beta_deg, True )
	>>> move_all_to( alpha_deg, beta_deg, True )

"""
    print(strg)
    return


def move_all_to( alpha_deg, beta_deg, calibrated=True ):
   move_to( 0, alpha_deg, beta_deg, calibrated=calibrated )
   return


def test_path( gd, gs, path_file, canmap_file, fpuset=[] ):
    # Execute a sequence of commands to test the paths contained in a
    # path file. Only valid if the wflib library has been imported.
    assert wflib is not None

    # First move to the starting position
    print("Moving FPUs to starting position...")
    gd.configZero(gs)
    gd.executeMotion(gs)

    # Now load the paths
    print("Configuring FPUs: %s" % str(fpuset))
    p = wflib.load_paths( path_file, canmap_file )
    gd.configPaths(p, gs, fpuset=fpuset )

    # If the paths have been loaded they can be executed and reversed
    print("Moving forwards along path...")
    gd.executeMotion(gs)
    print("Reversing path...")
    gd.reverseMotion(gs)
    gd.executeMotion(gs)


def get_arm_angles( gd, gs, fpuset=[] ):
    # Get the current arm angles in degrees.
    #
    # The function will try to use the tracked angles, which are based on the
    # motion history. If the interval returned by tracked angles contains a
    # scalar value, then the software and electronics are in agreement. The
    # angle is acceptable.
    #
    # If the tracked angles returns an interval containing a range of angles,
    # then the software's history is uncertain, but the angle is likely to be
    # within the interval returned. This is likely to happen after a collision
    # or emergency stop. If the motor step count contains an angle within the
    # expected interval, then the software and electronics are consistent. A
    # message is printed and the angle derived from the motor step count is
    # used.
    #
    # If the motor step count is not consistent with the range of angles
    # tracked by the software then the angle is unreliable. This might happen
    # if the electronics has been switched off. In this situation a warning
    # is issued and a metrology measurement can be used to derive better angles.
    #
    arm_angles = []
    
    # Get both the tracked version of the angles (base on motion history)
    # and the counted version of the angles (based on motor step count).
    tracked = gd.trackedAngles(gs, fpuset=fpuset, retrieve=True)
    counted = gd.countedAngles(gs, fpuset=fpuset, show_uninitialized=True)

    if fpuset:
        fpurange = fpuset
    else:
        fpurange = range(0, len(tracked) )

    ii = 0
    for track, angle_pair in zip(tracked, counted):
        fpuid = fpurange[ii]
        ii += 1
        # Extract the alpha and beta angles from Interval objects
        ainterval, binterval = track
        # Extract the counted angles
        alpha_counted, beta_counted = angle_pair

        try:
            # The tracked angles are OK as long as the alpha angle is not uncertain
            alpha_deg = float(ainterval.as_scalar())
        except AssertionError:
            # If the alpha range is uncertain, use the counted angle if consistent
            if alpha_counted >= ainterval.min() and alpha_counted <= ainterval.max():
                print("NOTE: FPU %d alpha angle is uncertain, %s, but motor step angle %s is valid." % (fpuid,str(ainterval),alpha_counted))
                alpha_deg = float(alpha_counted)
            else:
                strg = "WARNING: FPU %d alpha angle is uncertain, % " % (fpuid,str(ainterval))
                strg += "and motor step angle %s is invalid! " % alpha_counted
                strg += "Using average value."
                print(strg)
                alpha_deg = (ainterval.min() + ainterval.max())/2.0 # Or return NaN?
        try:
            # The tracked angles are OK as long as the beta angle is not uncertain
            beta_deg = float(binterval.as_scalar())
        except AssertionError:
            # If the beta range is uncertain, use the counted angle if consistent
            if beta_counted >= binterval.min() and beta_counted <= binterval.max():
                print("NOTE: FPU %d beta angle is uncertain, %s, but motor step angle %s is valid." % (fpuid,str(binterval),beta_counted))
                beta_deg = float(beta_counted)
            else:
                strg = "WARNING: FPU %d beta angle is uncertain, % " % (fpuid,str(binterval))
                strg += "and motor step angle %s is invalid! " % beta_counted
                strg += "Using average value."
                print(strg)
                beta_deg = (binterval.min() + binterval.max())/2.0 # Or return NaN?
            
        arm_angles.append( [fpuid, alpha_deg, beta_deg] )
    return arm_angles


def save_state_to_file( gd, gs, status_file, config_file, canmap_file="canmap.cfg",
                        fpuset=[] ): 

    # Retrieve the arm angles in degrees
    arm_angles = get_arm_angles( gd, gs, fpuset=fpuset )

    # Save those angles as target locations for the given configuration file
    wflib.save_angles_to_file( arm_angles, status_file, config_file,
                               canmap_file )


def recover_faults( gd, gs, fpuset=None, move_alpha=True, move_beta=True,
                    distance=1.0, verbose=False):
    # Execute a series of fault recovery procedures to try to bring the grid into a safe state
    import time

    nfaults = gd.countFaults( gs, fpuset=fpuset )
    if nfaults == 0:
        print("There are no faults to be recovered. No changes made.")
        return
    
    # First attempt to recover the faults
    print("Recovering faults...")
    (nfaults, last_pos, directions) = gd.recoverFaults(gs, fpuset=fpuset,
                                                       move_alpha=move_alpha,
                                                       move_beta=move_beta,
                                                       verbose=verbose)
    
    # Check that the faults really have been recovered and don't spontaneously
    # reappear because the FPUs are too close.
    # NOTE: Two attempts at recovering the faults is probably enough.
    time.sleep(1)
    nfaults = gd.countFaults( gs, fpuset=fpuset )
    if nfaults > 0:
        print("%d faults still remain. Recovering faults again..." % nfaults)
        (nfaults, last_pos, directions) = gd.recoverFaults(gs, fpuset=fpuset,
                                                          last_position=last_pos,
                                                          direction_needed=directions,
                                                          move_alpha=move_alpha,
                                                          move_beta=move_beta,
                                                          verbose=verbose)
        
    # If the faults have been recovered, move the FPUs a little further apart
    time.sleep(1)
    nfaults = gd.countFaults( gs, fpuset=fpuset )
    if nfaults == 0:
        print("All faults recovered. Moving the FPUs %.3f (deg) apart" % distance)
        gd.configMoveDir( gs, directions, distance=distance)
        gd.executeMotion( gs )
    

def dir_to_strg( direction ):
   # Helper function to convert a direction value into a string
   strg = "%1d=" % direction
   if direction == DIRST_CLOCKWISE:
      strg += "-CW"
   elif direction == DIRST_RESTING_LAST_CW:
      strg += "RCW"
   elif direction == DIRST_ANTI_CLOCKWISE:
      strg += "+AC"
   elif direction == DIRST_RESTING_LAST_ACW:
      strg += "RAC"
   else:
      strg += "???"
   return strg


#-------------------------------------------------------------------------------
def check_status( gs ):
    # Print a summary of the important status fields for each FPU.
    strg =  " ID    FPU       State           asteps bsteps adir  bdir  adatum bdatum aref  bref  alimit bcollision wfstatus wfvalid\n"
    strg += "---- ------ -------------------- ------ ------ ----- ----- ------ ------ ----- ----- ------ ---------- -------- -------\n"
#    for fpu_id in gd.getFpuIdList():
    for fpu_id in range(0, len(gs.FPU)):
       fpu = gs.FPU[fpu_id]
       strg += "%4d " % fpu_id
       strg += "%6s " % str(fpu.serial_number)
       strg += "%20s " % str(fpu.state)
       strg += "%6d " % int(fpu.alpha_steps)
       strg += "%6d " % int(fpu.beta_steps)
       strg += "%5s " % dir_to_strg(fpu.direction_alpha)
       strg += "%5s " % dir_to_strg(fpu.direction_beta)
       strg += "%5s  " % str(fpu.alpha_datum_switch_active)
       strg += "%5s  " % str(fpu.beta_datum_switch_active)
       strg += "%5s " % str(fpu.alpha_was_referenced)
       strg += "%5s  " % str(fpu.beta_was_referenced)
       strg += "%5s      " % str(fpu.at_alpha_limit)
       strg += "%5s     " % str(fpu.beta_collision)
       strg += "%4d   " % int(fpu.waveform_status)
       strg += "%5s  " % str(fpu.waveform_valid)
       strg += "\n"
    print(strg)



if __name__ == '__main__':

    print("Module version is:", FpuGridDriver.__version__, ", CAN PROTOCOL version:", FpuGridDriver.CAN_PROTOCOL_VERSION)

    if wflib is None:
       print("***wflib.load_paths function is not available until the mocpath module is installed.") 

    args = parse_args()

    gd, grid_state = initialize_FPU(args)
    gs = grid_state # alias grid_state to short name

    print("Issuing pingFPUs and getting positions:")
    gd.pingFPUs( grid_state )

    print("Important FPU status parameters:")
    check_status( grid_state )

    print("Tracked positions:")
    gd.trackedAngles( grid_state )

    clockwise_pars = dict([(k, SEARCH_CLOCKWISE) for k in range(args.N)])
    acw_pars = dict([(k, SEARCH_ANTI_CLOCKWISE) for k in range(args.N)])

    print("""If none of the FPUs are in an error state, you can issue now:

    gd.configDatum(grid_state)     # Only needed when FPU positions are not close to (-180,0)
    gd.executeMotion(grid_state)   # Only needed when FPU positions are not close to (-180,0)
    gd.findDatum(grid_state)

    to move FPUs to datum and initialise the grid. show_help() for more help. Ctrl/D to exit.""")
