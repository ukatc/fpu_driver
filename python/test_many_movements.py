from __future__ import print_function, division


import os
import argparse
import random

import FpuGridDriver
from FpuGridDriver import  DASEL_BOTH, DASEL_ALPHA, DASEL_BETA, \
    SEARCH_CLOCKWISE, SEARCH_ANTI_CLOCKWISE, SEARCH_AUTO, SKIP_FPU, \
    REQD_CLOCKWISE, REQD_ANTI_CLOCKWISE, REQD_NEGATIVE, REQD_POSITIVE, \
    DATUM_TIMEOUT_DISABLE

from fpu_commands import *

NUM_FPUS = int(os.environ.get("NUM_FPUS","10"))
NUM_TIMES = 10


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
                        help='Number of addressed FPUs (default: %(default)s).')

    parser.add_argument('-T', '--NUM_TIMES',  metavar='NUM_TIMES', dest='T', type=int, default=NUM_TIMES,
                        help='Number of times to repeat the test (default: %(default)s).')


    args = parser.parse_args()
    return args


def initialize_FPU(args):

    gd = FpuGridDriver.GridDriver(args.N, mockup=args.mockup)

    if args.mockup:
        gateway_address = [ FpuGridDriver.GatewayAddress("127.0.0.1", p)
                            for p in [4700, 4701, 4702] ]
    else:
        gateway_address = [ FpuGridDriver.GatewayAddress(args.gateway_address, args.gateway_port) ]

    print("connecting grid:", gd.connect(address_list=gateway_address))


    # We monitor the FPU grid by a variable which is
    # called grid_state, and reflects the state of
    # all FPUs.
    grid_state = gd.getGridState()

    if args.resetFPU:
        print("resetting FPU")
        gd.resetFPUs(grid_state)
        print("OK")

    return gd, grid_state


if __name__ == '__main__':

    print("module version is:", FpuGridDriver.__version__, ", CAN PROTOCOL version:", FpuGridDriver.CAN_PROTOCOL_VERSION)

    args = parse_args()

    gd, grid_state = initialize_FPU(args)
    gs = grid_state # alias grid_state to short name

    print("issuing pingFPUs and getting positions:")
    gd.pingFPUs(grid_state)

    print("tracked positions:")

    gd.trackedAngles(grid_state)



    clockwise_pars = dict([(k, SEARCH_CLOCKWISE) for k in range(args.N)])
    acw_pars = dict([(k, SEARCH_ANTI_CLOCKWISE) for k in range(args.N)])

#    print("""If all FPUs are shown with correct positions, you can issue now:
#
#    gd.findDatum(grid_state)
#
#    to move the FPUs to datum and initialise the grid.""")

    MIN_ALPHA = 0.1
    MAX_ALPHA = 150.0
    MIN_BETA  = -120.0
    MAX_BETA  = 120.0
    print(args.N, "FPUs will be moved randomly", args.T, "times.")

    for ii in range(0, args.T):
       ii1 = ii+1
       print("ITERATION %d/%d. Finding datum..." % (ii1,args.T))
       gd.findDatum(gs)

       print("ITERATION %d/%d. Status after findDatum" % (ii1,args.T))
       print(gs)

       alpha1 = round(random.uniform(MIN_ALPHA, MAX_ALPHA), 3)
       alpha2 = round(random.uniform(MIN_ALPHA, MAX_ALPHA), 3)
       alpha3 = round(random.uniform(MIN_ALPHA, MAX_ALPHA), 3)
       beta1 = round(random.uniform(MIN_BETA, MAX_BETA), 3)
       beta2 = round(random.uniform(MIN_BETA, MAX_BETA), 3)
       beta3 = round(random.uniform(MIN_BETA, MAX_BETA), 3)
       #alpha1 = 1.0
       #alpha2 = 20.0
       #alpha3 = 160.0
       #beta1 = -130.0
       #beta2 = 1.0
       #beta3 = 60.0

       print("Defining waveforms for (%f,%f) (%f,%f) (%f,%f) " % ( alpha1, beta1, alpha2, beta2, alpha3, beta3))
       wf = gen_wf([alpha1, alpha2, alpha3], [beta1, beta2, beta3])

       print("ITERATION %d/%d. Configuring motion." % (ii1,args.T))
       gd.configMotion(wf, gs)

       print("ITERATION %d/%d. Executing motion" % (ii1,args.T))
       gd.executeMotion(gs)

       print("ITERATION %d/%d. Status after movement" % (ii1,args.T))
       print(gs)

       print("ITERATION %d/%d. Reversing motion" % (ii1,args.T))
       gd.reverseMotion(gs)
       gd.executeMotion(gs)

       print("ITERATION %d/%d. Status after reverse" % (ii1,args.T))
       print(gs)
       print("")

    print("Finding datum one last time...")
    gd.findDatum(gs)

    print("Test finished successfully.")