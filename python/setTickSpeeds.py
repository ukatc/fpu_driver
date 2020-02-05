from __future__ import print_function, division

import os
import argparse
import time

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



    w = gen_wf(100,100)
    for ticks in [200, 250, 300]:
        gd.resetFPUs(gs)
        gd.setTicksPerSegment(ticks * 10000, gs)
        gd.findDatum(gs)
        gd.configMotion(w, gs)
        print("moving with ticks = {}".format(ticks))
        ti = time.time()
        gd.executeMotion(gs)
        gd.reverseMotion(gs)
        gd.executeMotion(gs)
        to = time.time()
        print("that took {:.3f} seconds".format(to - ti))

    gd.findDatum(gs)
    gd.resetFPUs(gs)
