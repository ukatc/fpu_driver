#!/usr/bin/env python

################################################################################
# initialiseGrid_NEW.py
#
# New version of initialiseGrid script, which uses the new C++ / Boost.Python-
# wrapped version of the grid driver functionality.
#
# NOTE: This script was modified for the new flexible CAN mapping functionality.
# TODO: Have put the text "FLEXIBLE_CAN_MAPPING" here temporarily, so that
# helps with searching for flexible CAN mapping changes across the codebase -
# remove this comment eventually. 
#
################################################################################

"""

This Python script provides an interactive interface where an expert user can
communicate with the FPU grid driver software. The script initialises the
grid by connecting to the EtherCAN interface, creating a grid state object
and pinging the FPUs, then it hands over to the user.

The script can be started by the command

    python -i initialiseGrid_NEW.py -F <CAN map file path> --gateway_address <IP address>

where the CAN map file path points to the CAN map text file which defines the list
of FPU IDs and their CAN routes, in 4 columns of numbers (without headings):
        <FPU ID> <Gateway ID> <Bus ID> <CAN ID>

For example:

    python -i initialiseGrid_NEW.py -F /moons/canmap1.csv --gateway_address 192.168.0.10

If EtherCAN hardware is not available, the script can communicate with the EtherCAN
simulator (mock gateway) with the command:

    python -i initialiseGrid_NEW.py -F <CAN map file path> --mockup

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

"""

from __future__ import print_function, division


import os
import argparse

import ethercanif
# TODO: The original initialiseGrid.py also imported REQD_NEGATIVE and
# REQD_POSITIVE, but these do not seem to be used anywhere, so not needed?
from ethercanif import DASEL_BOTH, DASEL_ALPHA, DASEL_BETA, \
    SEARCH_CLOCKWISE, SEARCH_ANTI_CLOCKWISE, SEARCH_AUTO, SKIP_FPU, \
    REQD_CLOCKWISE, REQD_ANTI_CLOCKWISE, DATUM_TIMEOUT_DISABLE

try:
   import wflib
except:
   wflib = None
from fpu_commands import *

NUM_FPUS = int(os.environ.get("NUM_FPUS","10"))

#-------------------------------------------------------------------------------
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

    parser.add_argument('-F', '--can_map_file',  metavar='CAN_MAP_FILE',
                        dest='F', type=str, help='Path of CAN map file')

    args = parser.parse_args()
    return args

#-------------------------------------------------------------------------------
def initialize_FPU(args):
    # Execute the common sequence of commands to initialize the FPU grid.
    gd = ethercanif.GridDriver()

    gd.initialize(canmapfile = args.F, mockup = args.mockup)

    if args.mockup:
        gateway_address = [ ethercanif.GatewayAddress("127.0.0.1", p)
                            for p in [4700, 4701, 4702] ]
    else:
        gateway_address = [ ethercanif.GatewayAddress(args.gateway_address,
                                                      args.gateway_port) ]

    print("Connecting grid:", gd.connect(address_list=gateway_address))

    # We monitor the FPU grid by a variable which is called grid_state, which
    # reflects the state of all FPUs.
    grid_state = gd.getGridState()

    if args.resetFPU:
        print("Resetting FPUs")
        gd.resetFPUs(grid_state)
        print("OK")

    return gd, grid_state

#-------------------------------------------------------------------------------
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

#-------------------------------------------------------------------------------
def check_status( gs ):
    # Print a summary of the important status fields for each FPU.
    strg =  " ID    FPU       State           asteps bsteps adatum bdatum  aref  bref alimit bcollision wfstatus wfvalid\n"
    strg += "---- ------ -------------------- ------ ------ ------ ------ ----- ----- ------ ---------- -------- -------\n"
    for fpu_id in gd.getFpuIdList():
       fpu = gs.FPU[fpu_id]
       strg += "%4d " % fpu_id
       strg += "%6s " % str(fpu.serial_number)
       strg += "%20s " % str(fpu.state)
       strg += "%6d " % int(fpu.alpha_steps)
       strg += "%6d  " % int(fpu.beta_steps)
       strg += "%5s  " % str(fpu.alpha_datum_switch_active)
       strg += "%5s " % str(fpu.beta_datum_switch_active)
       strg += "%5s " % str(fpu.alpha_was_referenced)
       strg += "%5s  " % str(fpu.beta_was_referenced)
       strg += "%5s      " % str(fpu.at_alpha_limit)
       strg += "%5s     " % str(fpu.beta_collision)
       strg += "%4d   " % int(fpu.waveform_status)
       strg += "%5s  " % str(fpu.waveform_valid)
       strg += "\n"
    print(strg)

#-------------------------------------------------------------------------------
if __name__ == '__main__':

    print("Module version is:", ethercanif.__version__, ", CAN PROTOCOL version:",\
          ethercanif.CAN_PROTOCOL_VERSION)

    if wflib is None:
       print("***wflib.load_paths function is not available until the mocpath module is installed.") 

    args = parse_args()

    gd, grid_state = initialize_FPU(args)
    gs = grid_state # alias grid_state to short name

    print("Issuing pingFPUs and getting positions:")
    gd.pingFPUs(grid_state)

    print("Important FPU status parameters:")
    check_status(grid_state)

    print("Tracked positions:")
    gd.trackedAngles(grid_state)

    clockwise_pars = dict([(k, SEARCH_CLOCKWISE) for k in gd.getFpuIdList()])
    acw_pars = dict([(k, SEARCH_ANTI_CLOCKWISE) for k in gd.getFpuIdList()])

    print("""If none of the FPUs are in an error state, you can issue now:

    gd.configDatum(grid_state)     # Only needed when FPU positions are not close to (-180,0)
    gd.executeMotion(grid_state)   # Only needed when FPU positions are not close to (-180,0)
    gd.findDatum(grid_state)

    to move the FPUs to datum and initialise the grid. Ctrl/D to exit.""")

#-------------------------------------------------------------------------------

