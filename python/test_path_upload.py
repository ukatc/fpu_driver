from __future__ import print_function, division

import os
import argparse

import FpuGridDriver
from FpuGridDriver import  DASEL_BOTH, DASEL_ALPHA, DASEL_BETA, \
    SEARCH_CLOCKWISE, SEARCH_ANTI_CLOCKWISE, SEARCH_AUTO, SKIP_FPU, \
    REQD_CLOCKWISE, REQD_ANTI_CLOCKWISE, DATUM_TIMEOUT_DISABLE

from fpu_commands import *
from fpu_constants import *
from wflib import load_waveform

NUM_FPUS = int(os.environ.get("NUM_FPUS","7"))


def parse_args():
    parser = argparse.ArgumentParser(description='set up grid to upload waveform from path generator')
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

    parser.add_argument('--min_step_frequency', metavar='min_step_frequency', type=float,
                       # default=MOTOR_MIN_STEP_FREQUENCY,
                       default=490.0,
                        help='minimum motor step frequency  (default: %(default)s)')
        
    parser.add_argument('--max_step_frequency', metavar='max_step_frequency', type=float,
                       default=MOTOR_MAX_STEP_FREQUENCY,
                        help='maximum motor step frequency  (default: %(default)s)')
        
    parser.add_argument('--max_start_frequency', metavar='max_start_frequency', type=float,
                        default=MOTOR_MAX_START_FREQUENCY,
                        help='maximum motor start frequency  (default: %(default)s)')
        
        
    parser.add_argument('--max_acceleration', metavar='MAX_ACCELERATION', type=float,
                        #default=MAX_ACCELERATION_FACTOR,
                        default=1.6,
                        help='maximum motor acceleration  (default: %(default)s)')

    parser.add_argument('--segment_length_ms', metavar='SEGMENT_LENGTH_MS', type=float,
                        default=WAVEFORM_SEGMENT_LENGTH_MS,
                        help='waveform segment length, in milliseconds (default: %(default)s)')
    
    args = parser.parse_args()

    args.min_steps=int(args.min_step_frequency * args.segment_length_ms / 1000)
    args.max_steps=int(ceil(args.max_step_frequency * args.segment_length_ms / 1000))
    args.start_steps=int(ceil(args.max_start_frequency * args.segment_length_ms / 1000))

    return args


def initialize_FPU(args):
    
    gd = FpuGridDriver.GridDriver(args.N,
                                  motor_minimum_frequency=args.min_step_frequency,  
                                  motor_maximum_frequency=args.max_step_frequency, 
                                  motor_max_start_frequency=args.max_start_frequency,
                                  motor_max_rel_increase=args.max_acceleration)

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




    


