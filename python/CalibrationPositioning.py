#!/usr/bin/python
from __future__ import print_function
"""This module drives one FPU to a fine-grained sequence of 
positions and calls a measurement function at each position.
"""

import time
import argparse
import numpy

import FpuGridDriver
from FpuGridDriver import TEST_GATEWAY_ADRESS_LIST, GatewayAddress
from fpu_commands import *

NUM_FPUS = 1






def dummy_metrology_func(alpha_steps, beta_steps):
    print("now performing dummy measurement at"
          " alpha={}, beta={} steps".format(alpha_steps,beta_steps))
    ## insert actual measurement stuff here
    time.sleep(1)
    

def measure_position(gd, grid_state, alpha, beta, metrology_func=None,
                     return_to_datum=True, deviation_list=[]):
    
    alpha0, beta0 = list_angles(grid_state)[0]
    alpha_move = alpha - alpha0
    beta_move = beta - beta0
    
    # generate waveform
    waveform = gen_wf(alpha_move, beta_move)

    # configure waveform, by uploading it to the FPU
    gd.configMotion(waveform, grid_state)

    print("moving to ({},{})".format(alpha, beta))
    gd.executeMotion(grid_state)

    # display the new position
    print("measuring at position:", list_angles(grid_state)[0])

    alpha_steps = grid_state.FPU[0].alpha_steps
    beta_steps = grid_state.FPU[0].beta_steps
    # call metrology function for measurement
    metrology_func(alpha_steps, beta_steps)


    if return_to_datum:
        if (alpha0 == 0) and (beta0 == 0):
            gd.reverseMotion(grid_state)
            gd.executeMotion(grid_state)
        else:
            waveform = gen_wf(-alpha, -beta)
            gd.configMotion(waveform, grid_state)
            

        gd.getCounterDeviation(grid_state)

        # we get the state for FPU 0 (the only one)
        fpu_state = grid_state.FPU[0]
        print("counter deviation: "
              "(dev_alpha, dev_beta) = ({}, {}) steps".format(
                  fpu_state.alpha_deviation,
                  fpu_state.beta_deviation))
        
        deviation_list.append( (alpha, beta,
                                fpu_state.alpha_deviation,
                                fpu_state.beta_deviation))

      


def parse_args():
    parser = argparse.ArgumentParser(description='Drive FPU to a grid of positions and call measurement function')
    parser.add_argument('--mockup',   default=False, action='store_true',
                        help='set gateway address to use mock-up gateway and FPU')

    parser.add_argument('--resetFPU',   default=False, action='store_true',
                        help='reset FPU so that earlier aborts / collisions are ignored')

    parser.add_argument('--gateway_port', metavar='GATEWAY_PORT', type=int, default=4700,
                        help='EtherCAN gateway port number (default: %(default)s)')

    parser.add_argument('--gateway_address', metavar='GATEWAY_ADDRESS', type=str, default="192.168.0.10",
                        help='EtherCAN gateway IP address or hostname (default: %(default)r)')
    
    parser.add_argument('--alpha_min', metavar='ALPHA_MIN', type=float, default=0.0,
                        help='minimum alpha value  (default: %(default)s)')
    parser.add_argument('--alpha_max', metavar='ALPHA_MAX', type=float, default=270.0,
                        help='maximum alpha value  (default: %(default)s)')
    parser.add_argument('--beta_min', metavar='BETA_MIN', type=float, default=0.0,
                        help='minimum beta value  (default: %(default)s)')
    parser.add_argument('--beta_max', metavar='BETA_MAX', type=float, default=180.0,
                        help='maximum beta value  (default: %(default)s)')
    
    parser.add_argument('asteps', metavar='ASTEPS', type=int, default=10,
                        help='number of alpha steps  (default: %(default)s)')
    parser.add_argument('bsteps', metavar='BSTEPS', type=int, default=10,
                        help='number of beta steps  (default: %(default)s)')
    
    parser.add_argument('--datum_at', metavar='DATUM_AT', type=str, default="start",
                        choices=['alpha_change', 'beta_change', 'start'],
                        help='go to datum at change of alpha / beta coordinate (default: %(default)r)')
    
    args = parser.parse_args()
    return args



def initialize_FPU(args):
    
    gd = FpuGridDriver.GridDriver(NUM_FPUS)

    print("initializing driver: ", gd.initializeDriver())


    if args.mockup:
        gateway_address = [ GatewayAddress("127.0.0.1", 4700)  ]
    else:
        gateway_address = [ GatewayAddress(args.gateway_address, args.gateway_port) ]

    print("connecting grid:", gd.connect(address_list=gateway_address))


    # We monitor the FPU grid by a variable which is
    # called grid_state, and reflects the state of
    # all FPUs.
    grid_state = gd.getGridState()

    if args.resetFPU:
        print("resetting FPU")
        gd.resetFPUs(grid_state)
        print("OK")

    # Now, we issue a findDatum method. In order to know when and how
    # this command finished, we pass the grid_state variable.
    print("issuing findDatum:")
    gd.findDatum(grid_state)
    print("findDatum finished")

    # We can use grid_state to display the starting position
    print("the starting position (in degrees) is:", list_angles(grid_state)[0])

    return gd, grid_state


def loop_positions(args, gd, grid_state, metrology_func=None, deviation_list=[]):
                   
    for alpha in numpy.linspace(args.alpha_min, args.alpha_max, args.asteps):
        if args.datum_at == 'alpha_change':
            go_datum = True
        else:
            go_datum = False
            
        for beta in numpy.linspace(args.beta_min, args.beta_max, args.bsteps):
            if args.datum_at == 'beta_change':
                go_datum = True
                
            print("measuring at ({},{}), datum={}".format(alpha, beta, go_datum))
            measure_position(gd, grid_state, alpha, beta, return_to_datum=go_datum,
                             metrology_func=metrology_func, deviation_list=deviation_list)
        
            go_datum = False


                
if __name__ == '__main__':
    # parse arguments
    args = parse_args()
    
    # initialize FPU accordingly
    gd, grid_state = initialize_FPU(args)

    deviations = []

    # loop over configured positions, calling metrology_func for
    # each position
    loop_positions(args, gd, grid_state, metrology_func=dummy_metrology_func,
                   deviation_list=deviations)
    
    print("counter deviations:", deviations)

    print("ready.")
                    

              





