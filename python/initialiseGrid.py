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

The following commands will display the state of the FPU grid

>>> plot_status( gd, gs, config_file, canmap_fname)

The following command can be used to recover from a fault or collision

>>> recover_faults( gd, gs, config_file, canmap_fname, fpuset=[]
                    use_firmware_directions=False, max_steps=6, distance=1.0,
                    verbose=False)
                    
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

from fpu_constants import RADIAN_TO_DEGREE, DEGREE_TO_RADIAN
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


def check_firmware_vs_database( gd, gs, fpuset=None ):
    """

    Check that the last known positions stored in the database (tracked angles)
    are consistent with the step counts reported by the firmware (counted_list).

    Returns True if consistent.

    """
    consistent = True
    tracked_list = gd.trackedAngles(gs, fpuset=fpuset, retrieve=True, display=False)
    counted_list = gd.countedAngles(gs, fpuset=fpuset, show_uninitialized=True)

    for (tracked, counted) in zip(tracked_list, counted_list):
        # Check the alpha angle is within the expected range
        if not (tracked[0].min() <= counted[0] <= tracked[0].max()):
            #print("Alpha angle out of range: %f < %f < %f" % \
            #   (tracked[0].min(), counted[0], tracked[0].max()))
            consistent = False
        # Check the beta angle is within the expected range
        if not (tracked[1].min() <= counted[1] <= tracked[1].max()):
            #print("Beta angle out of range: %f < %f < %f" % \
            #   (tracked[1].min(), counted[1], tracked[1].max()))
            consistent = False
    return consistent

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


def get_arm_angles( gd, gs, fpuset=[], use_step_counts=False,
                    convert_to_radians=False ):
    # Get the current arm angles in degrees or radians.
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

        if use_step_counts:
            # Just use the step counts reported by the firmware and ignore
            # the tracked angles.
            alpha_deg = float(alpha_counted)
            beta_deg = float(beta_counted)
        else:
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
        if convert_to_radians:
            arm_angles.append( [fpuid, alpha_deg * DEGREE_TO_RADIAN, beta_deg * DEGREE_TO_RADIAN] )
        else:
            arm_angles.append( [fpuid, alpha_deg, beta_deg] )
    return arm_angles


def save_state_to_file( gd, gs, status_file, config_file, canmap_file="canmap.cfg",
                        fpuset=[] ): 

    # Retrieve the arm angles in degrees
    arm_angles = get_arm_angles( gd, gs, fpuset=fpuset )

    # Save those angles as target locations for the given configuration file
    wflib.save_angles_to_file( arm_angles, status_file, config_file,
                               canmap_file )

def plot_status( gd, gs, config_file, canmap_fname, title=""):
    # Plot the current grid status
    if wflib is not None:
        arm_angles = get_arm_angles( gd, gs, convert_to_radians=True)
        wflib.plot_geometry( config_file, canmap_fname, arm_angles, title=title )
    else:
        print("No geometry functions available.")

def move_to_datum( gd, gs, soft_protection=True, check_protection=None,
                    allow_uninitialized=True, use_step_counts=True ):
    # Package up the two commands needed to move to datum
    gd.configDatum( gs, soft_protection=soft_protection,
                    check_protection=check_protection,
                    allow_uninitialized=allow_uninitialized,
                    use_step_counts=use_step_counts )
    gd.executeMotion( gs )
    
def move_to_zero( gd, gs, soft_protection=True, check_protection=None,
                   allow_uninitialized=False ):
    # Package up the two commands needed to move to the zero point
    gd.configZero( gs, soft_protection=soft_protection,
                   check_protection=check_protection,
                   allow_uninitialized=allow_uninitialized )
    gd.executeMotion( gs )

def move_path( gd, gs, path_file, canmap_file, fpuset=None,
               soft_protection=True, check_protection=None,
               allow_uninitialized=False, reverse=False ):
    # Package up the three commands needed to move along a path
    p = wflib.load_paths( path_file, canmap_fname, reverse=reverse)
    gd.configPaths( p, gs, fpuset=fpuset, soft_protection=soft_protection,
                    check_protection=check_protection,
                    allow_uninitialized=allow_uninitialized, reverse=reverse )
    gd.executeMotion( gs )

def move_to_target( gd, gs, config_file, canmap_fname, target, fpuset=None, bf=64, verbose=False):
    # Move fibre positioners to the given named target.
    # Returns True if completely successful. 
    arm_angles = get_arm_angles( gd, gs, fpuset=fpuset,
                                 convert_to_radians=True)
    (target_status, safe_paths) = wflib.generate_safe_paths( config_file,
                                            canmap_fname,
                                            arm_angles, 
                                            target=target,
                                            brute_force_elements=bf )
    if safe_paths is not None:
        # Convert the paths into grid driver format.
        new_paths = wflib.convert_paths( safe_paths,
                                         canmap_fname=canmap_fname)

        # Load the new paths into the grid driver and execute them
        if target == "SPACE":
            print("Moving positioners apart...")
        elif bf < 130:
            print("Moving safely to %s..." % target)
        else:
            print("Moving directly to %s..." % target)

        gd.configPaths(new_paths, gs, allow_uninitialized=True)
        gd.executeMotion(gs)
        if target_status[1] == target_status[0]:
            # All FPUs have successfully reached their target
            return True
        else:
            # At least one FPU has deadlocked. More iterations needed.
            return False
    else:
        print("ERROR: Could not generate safe paths.")
        return False

def make_safe( gd, gs, config_file, canmap_fname, fpuset=None,
               targets=["SPACE","DEFAULT","DEFAULT","SAFE","DEFAULT"],
               bf=[64,64,256,128,256], verbose=False):
    # The target parameter must be a string or a list
    if isinstance( targets, str):
        targets = [targets]
    if len(targets[0]) < 1:
        targets=["DEFAULT"]
    if isinstance( bf, int):
        bf = [bf] * len(targets)
 
    # Move the fibre positioners to a safe location after a fault
    if (config_file is not None) and (wflib is not None):
        ii = 1
        for target, b in zip( targets, bf ):
            print("Pass %d for target %s (bf=%d). Analyzing geometry..." % \
                (ii, target, b))
            result = move_to_target( gd, gs, config_file, canmap_fname, target,
                                     fpuset=None, bf=b, verbose=verbose )
            ii += 1
            if result:
                # Stop iterating when all FPUs have reached their target.
                break
        if result:
            print("All FPUs safely recovered. They can now be moved to datum.")
        else:
            print("WARNING: Some FPUs could not be recovered. " + \
                  "Repeat the procedure or move the remaining ones manually.")
    else:
        # Paths cannot be generated without wflib
        print("ERROR: wflib functions not available. Paths cannot be generated.")

def recover_faults( gd, gs, config_file, canmap_fname, fpuset=None,
                    use_firmware_directions=False, max_steps=6,
                    max_attempts=2, distance=0.5, verbose=False):
    # Execute a series of fault recovery procedures to try to bring the grid into a safe state
    import time

    nfaults = gd.countFaults( gs, fpuset=fpuset )
    if nfaults == 0:
        print("There are no faults to be recovered. No changes made.")
        return

    if (config_file is not None) and (wflib is not None):
        print("Analyzing geometry...")
        arm_angles = get_arm_angles( gd, gs, fpuset=fpuset,
                                     convert_to_radians=True)
        directions = wflib.recovery_directions(
                                    config_file, canmap_fname,
                                    arm_angles, max_steps=max_steps,
                                    verbose=verbose )
        for fpuid in list(directions.keys()):
            this_dir = directions[fpuid]
            print("FPU %d: From geometry, alpha,beta directions are %s,%s" % \
                  (fpuid, this_dir[0], this_dir[2]))
    else:
        # If there are no mocpath directions, at least use the firmware directions
        print("WARNING: Geometry not available. Can only use info from firmware")
        use_firmware_directions = True
        directions = None

    # First attempt to recover the faults
    print("Recovering faults...")
    nattempts = 1
    (nfaults, last_pos) = gd.recoverFaults(
                            gs, directions, fpuset=fpuset,
                            use_firmware_directions=use_firmware_directions,
                            verbose=verbose )
    
    # Check that the faults really have been recovered and don't spontaneously
    # reappear because the FPUs are too close.
    # NOTE: Two attempts at recovering the faults is probably enough.
    time.sleep(1)
    nfaults = gd.countFaults( gs, fpuset=fpuset )
    while (nattempts < max_attempts) and (nfaults > 0):
        nattempts += 1
        print("%d faults still remain. Recovering faults again..." % nfaults)
        (nfaults, last_pos) = gd.recoverFaults(grid_state,
                                directions, fpuset=fpuset,
                                use_firmware_directions=use_firmware_directions,
                                verbose=verbose)
        
    # If the faults have been recovered, move the FPUs a little further apart
    time.sleep(1)
    nfaults = gd.countFaults( gs, fpuset=fpuset )
    if nfaults == 0 and directions is not None:
        print("All faults recovered. Moving the FPUs %.3f (deg) apart" % distance)
        gd.configMoveDir( gs, directions, distance=distance,
                          allow_uninitialized=True, use_step_counts=True)
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
    # Begin with default configuration file and canmap file names
    # NOTE: These definitions are just a time saver. These files are not guaranteed to exist.
    config_file = "/home/jnix/FPU6TestsMar2022/mpmcfgINSfps_6_ORIENT142.cfg"
    canmap_fname = "canmap6.cfg"

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

    if not check_firmware_vs_database( gd, gs ):
        strg = """
*** WARNING: Last known positions recalled from the database are inconsistent with the
    step counts reported by the firmware. There might have been a power cycle since you
    last used this software.
    - If you know the FPUs are in a safe state, find the datum, as described below.
    - If you do not know the state of the FPUs, use the metrology cameras to update the
      database and start again.
"""
        print(strg)

    clockwise_pars = dict([(k, SEARCH_CLOCKWISE) for k in range(args.N)])
    acw_pars = dict([(k, SEARCH_ANTI_CLOCKWISE) for k in range(args.N)])

    print("""If none of the FPUs are in an error state, you can issue now:

    gd.configDatum(grid_state)     # Only needed when FPU positions are not close to (-180,-6.5)
    gd.executeMotion(grid_state)   # Only needed when FPU positions are not close to (-180,-6.5)
    gd.findDatum(grid_state)

    to move FPUs to datum and initialise the grid. show_help() for more help. Ctrl/D to exit.""")
