################################################################################
# initialiseGrid_NEW.py
#
# New version of initialiseGrid script, which uses the new C++ / Boost.Python-
# wrapped version of the grid driver functionality.
#
################################################################################

from __future__ import print_function, division


import os
import argparse

import ethercanif
from ethercanif import DASEL_BOTH, DASEL_ALPHA, DASEL_BETA, \
    SEARCH_CLOCKWISE, SEARCH_ANTI_CLOCKWISE, SEARCH_AUTO, SKIP_FPU, \
    REQD_CLOCKWISE, REQD_ANTI_CLOCKWISE, DATUM_TIMEOUT_DISABLE
# TODO: The original initialiseGrid.py also imported REQD_NEGATIVE and
# REQD_POSITIVE, but these do not seem to be used anywhere, so not needed?
# (and there aren't Boost.Python definitions for them yet)

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

    parser.add_argument('-N', '--NUM_FPUS',  metavar='NUM_FPUS', dest='N', type=int, default=NUM_FPUS,
                        help='Number of adressed FPUs (default: %(default)s).')

    args = parser.parse_args()
    return args

#-------------------------------------------------------------------------------
def initialize_FPU(args):

    gd = ethercanif.GridDriver(nfpus = args.N)

    gd.initialize(mockup = args.mockup)

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

    print("Tracked positions:")
    gd.trackedAngles(grid_state)

    clockwise_pars = dict([(k, SEARCH_CLOCKWISE) for k in range(args.N)])
    acw_pars = dict([(k, SEARCH_ANTI_CLOCKWISE) for k in range(args.N)])

    print("""If all FPUs are shown with correct positions, you can issue now:

    gd.findDatum(grid_state)

    to move the FPUs to datum and initialise the grid.""")

#-------------------------------------------------------------------------------

