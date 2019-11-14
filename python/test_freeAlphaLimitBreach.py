from __future__ import print_function, division

import os
import argparse

import FpuGridDriver
from FpuGridDriver import  DASEL_BOTH, DASEL_ALPHA, DASEL_BETA, \
    SEARCH_CLOCKWISE, SEARCH_ANTI_CLOCKWISE, SEARCH_AUTO, SKIP_FPU, \
    REQD_CLOCKWISE, REQD_ANTI_CLOCKWISE, DATUM_TIMEOUT_DISABLE, LimitBreachError

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

def print_steps(gs):
    print("step counts = ({},{})".format(gs.FPU[0].alpha_steps,gs.FPU[0].beta_steps))


if __name__ == '__main__':

    print("module version is:", FpuGridDriver.__version__, ", CAN PROTOCOL version:", FpuGridDriver.CAN_PROTOCOL_VERSION)

    args = parse_args()

    gd, grid_state = initialize_FPU(args)
    gs = grid_state # alias grid_state to short name

    print("issuing pingFPUs and getting positions:")
    gd.pingFPUs(grid_state)

    print("tracked positions:")

    gd.trackedAngles(grid_state)



    gd.findDatum(gs)

    w = gen_wf(-1.0, 0.0)
    gd.configMotion(w, gs, soft_protection=False)
    print("before move")
    gd.trackedAngles(gs)
    print_steps(gs)
    gd.countedAngles(gs)

    print("moving to (-1,0) degrees")
    try:
        gd.executeMotion(gs)
    except LimitBreachError:
        print("limit breach detected")

    print("after move & breach")
    gd.trackedAngles(gs, show_offsets=True)
    print_steps(gs)
    #gd.countedAngles(gs)

    print("freeAlphaLimitBreach 1")
    gd.freeAlphaLimitBreach(0, REQD_ANTI_CLOCKWISE, gs)
    print("after freeAlphaLB 1")
    gd.trackedAngles(gs, show_offsets=True)
    print_steps(gs)
    #gd.countedAngles(gs)

    print("freeAlphaLimitBreach 2")
    gd.freeAlphaLimitBreach(0, REQD_ANTI_CLOCKWISE, gs)
    print("after freeAlphaLB 2")
    gd.trackedAngles(gs, show_offsets=True)
    print_steps(gs)
    #gd.countedAngles(gs)

    print("freeAlphaLimitBreach 3")
    gd.freeAlphaLimitBreach(0, REQD_ANTI_CLOCKWISE, gs)
    print("after freeAlphaLB 3")
    gd.trackedAngles(gs, show_offsets=True)
    print_steps(gs)
    #gd.countedAngles(gs)
    gd.enableAlphaLimitProtection(gs)


    w = gen_wf(1.0, 0.0)
    gd.configMotion(w, gs, soft_protection=False, allow_uninitialized=True)
    print("before back move")
    gd.trackedAngles(gs, show_offsets=True)
    print_steps(gs)
    #gd.countedAngles(gs)

    print("move by (1, 0) degrees")
    gd.executeMotion(gs)
    print("after back move")
    gd.trackedAngles(gs, show_offsets=True)
    print_steps(gs)
    #gd.countedAngles(gs)
