from __future__ import print_function, division

import os
import argparse

import FpuGridDriver
from FpuGridDriver import  DASEL_BOTH, DASEL_ALPHA, DASEL_BETA, \
    SEARCH_CLOCKWISE, SEARCH_ANTI_CLOCKWISE, SEARCH_AUTO, SKIP_FPU, \
    REQD_CLOCKWISE, REQD_ANTI_CLOCKWISE, DATUM_TIMEOUT_DISABLE

from fpu_commands import *

NUM_FPUS = int(os.environ.get("NUM_FPUS","10"))


def parse_args():
    parser = argparse.ArgumentParser(description='Ping a configured number of FPUs')
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

    gd = FpuGridDriver.GridDriver(args.N, mockup=args.mockup)

    if args.mockup:
        gateway_address = [ FpuGridDriver.GatewayAddress("127.0.0.1", p)
                            for p in [4700, 4701, 4702] ]
    else:
        gateway_address = [ FpuGridDriver.GatewayAddress(args.gateway_address, args.gateway_port) ]

    print("Connecting grid:", gd.connect(address_list=gateway_address))


    # We monitor the FPU grid by a variable which is
    # called grid_state, and reflects the state of
    # all FPUs.
    grid_state = gd.getGridState()

    if args.resetFPU:
        print("Resetting FPU")
        gd.resetFPUs(grid_state)
        print("OK")

    return gd, grid_state


if __name__ == '__main__':

    print("Module version is:", FpuGridDriver.__version__, ", CAN PROTOCOL version:", FpuGridDriver.CAN_PROTOCOL_VERSION)

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






    gd.findDatum(gs)
    w = gen_wf(350,0)
    gd.configMotion(w, gs)

    gd.executeMotion(gs)

    gd.resetFPUs(gs)
    w = gen_wf(-300,0)
    gd.configMotion(w, gs, allow_uninitialized=True)

    gd.executeMotion(gs)

    gd.trackedAngles(gs, show_offsets=True)

    w = gen_wf(10,0)
    gd.configMotion(w, gs, allow_uninitialized=True)

    gd.executeMotion(gs)
    gd.trackedAngles(gs, show_offsets=True)

    w = gen_wf(-20,0)
    gd.configMotion(w, gs, allow_uninitialized=True)

    gd.executeMotion(gs)
    gd.trackedAngles(gs, show_offsets=True)
    w = gen_wf(240,0)
    gd.configMotion(w, gs, allow_uninitialized=True)

    gd.executeMotion(gs)
    gd.trackedAngles(gs, show_offsets=True)
    w = gen_wf(-30,0)
    gd.configMotion(w, gs, allow_uninitialized=True)

    gd.executeMotion(gs)
    gd.trackedAngles(gs, show_offsets=True)

    gd.findDatum(gs, timeout=DATUM_TIMEOUT_DISABLE)
