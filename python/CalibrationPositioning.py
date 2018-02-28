#!/usr/bin/python
from __future__ import print_function, division
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

def printtime():
    print(time.strftime("%a, %d %b %Y %H:%M:%S +0000 (%Z)", time.gmtime()))
    

def measure_position(gd, grid_state, alpha, beta, metrology_func=None,
                     return_to_datum=True, deviation_list=[]):

    #gd.pingFPUs(grid_state)
    alpha0, beta0 = list_angles(grid_state)[0]
    print("current pos: ({},{})".format(alpha0, beta0))
    alpha_move = alpha - alpha0
    beta_move = beta - beta0

    if (alpha_move != 0) or (beta_move != 0):
        # generate waveform
        waveform = gen_wf(alpha_move, beta_move)

        # configure waveform, by uploading it to the FPU
        gd.configMotion(waveform, grid_state)

        printtime()
        print("now moving to ({},{})".format(alpha, beta))
        gd.executeMotion(grid_state)
    else:
        print("not moving (null movement discarded)")

    # display the new position
    #gd.pingFPUs(grid_state)
    print("measuring at position:", list_angles(grid_state)[0])

    alpha_steps = grid_state.FPU[0].alpha_steps
    beta_steps = grid_state.FPU[0].beta_steps
    # call metrology function for measurement
    metrology_func(alpha_steps, beta_steps)


    if return_to_datum:
        printtime()
        print("now moving to ({},{})".format(0, 0))
        if (alpha0 == 0) and (beta0 == 0):
            if (alpha_move != 0) or (beta_move != 0):
                gd.reverseMotion(grid_state)
                gd.executeMotion(grid_state)
            else:
                print("not moving (null movement discarded)")
        else:
            if (alpha != 0) or (beta != 0):
                waveform = gen_wf(-alpha, -beta)
                gd.configMotion(waveform, grid_state)
                gd.executeMotion(grid_state)
            else:
                print("not moving (null movement discarded)")

        printtime()
        print("position:", list_angles(grid_state), "steps=", list_positions(grid_state))
        print("issuing findDatum()")
        gd.findDatum(grid_state)
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
    parser.add_argument('--alpha_max', metavar='ALPHA_MAX', type=float, default=360.0,
                        help='maximum alpha value  (default: %(default)s)')
    parser.add_argument('--beta_min', metavar='BETA_MIN', type=float, default=-180.0,
                        help='minimum beta value  (default: %(default)s)')
    parser.add_argument('--beta_max', metavar='BETA_MAX', type=float, default=130.0,
                        help='maximum beta value  (default: %(default)s)')

    parser.add_argument('--chill_time', metavar='CHILL_TIME', type=float, default=1,
                        help='chill time for alpha arm  (default: %(default)s)')

    parser.add_argument('asteps', metavar='ASTEPS', type=int, default=10,
                        help='number of alpha steps  (default: %(default)s)')
    parser.add_argument('bsteps', metavar='BSTEPS', type=int, default=10,
                        help='number of beta steps  (default: %(default)s)')
    
    parser.add_argument('--pattern', metavar='PATTERN', type=str, default="abcircle",
                        choices=['abcircle', 'whitenoise', 'bluenoise', 'polargrid'],
                        help="""Pattern of movement. Available patterns are:

                        'abcircle' : Calibration pattern as described in VLT-TRE-MON-14620-3007, 
                                     section 5.1.6.
                        'polargrid': alpha and beta coordinates are stepped through in a grid 
                                     pattern, with the alpha angle moving after each beta angle
                                     was reached for one alpha position.

                        'whitenoise' : In this pattern, a new (alpha, beta) pair of angles 
                                     between the minimum and maximum values is drawn
                                     independently for each step.
                        'bluenoise' : In this pattern, new angles are drawn for each step so
                                      that the distribution of distance follows a triangular PDF.

                        default: %(default)r)""")
    
    parser.add_argument('--datum_at', metavar='DATUM_AT', type=str, default="start",
                        choices=['alpha_change', 'beta_change', 'start'],
                        help="go to datum at change of alpha / beta coordinate ('alpha_change', 'beta_change', 'start' default: %(default)r)")
    
    args = parser.parse_args()
    return args



def initialize_FPU(args):
    
    gd = FpuGridDriver.GridDriver(NUM_FPUS)

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


def grid_positions(args):
                   
    for alpha in numpy.linspace(args.alpha_min, args.alpha_max, args.asteps):
        if args.datum_at == 'alpha_change':
            go_datum = True
        else:
            go_datum = False
            
        for beta in numpy.linspace(args.beta_min, args.beta_max, args.bsteps):
            if args.datum_at == 'beta_change':
                go_datum = True

            yield (alpha, beta, go_datum)        
            go_datum = False
            

def iterate_positions(args, seq, gd, grid_state, metrology_func=None, deviation_list=[]):
                   
    for alpha, beta, go_datum in seq:
        print("measuring at ({},{}), datum={}".format(alpha, beta, go_datum))
        measure_position(gd, grid_state, alpha, beta, return_to_datum=go_datum,
                         metrology_func=metrology_func, deviation_list=deviation_list)
        
        if args.chill_time > 10:
            print("waiting %f seconds for fpu to cool off" % args.chill_time)
        time.sleep(args.chill_time)
            
    # last call at (0,0), and always return to datum
    measure_position(gd, grid_state, 0, 0, return_to_datum=True,
                     metrology_func=metrology_func, deviation_list=deviation_list)
    

def rungrid(args):
    # initialize FPU accordingly
    gd, grid_state = initialize_FPU(args)

    deviations = []

    # select position pattern
    if args.pattern == 'polargrid':
        poslist = grid_positions(args)
    else:
        raise ValueError("pattern not implemented")

    # loop over configured positions, calling metrology_func for
    # each position
    try:
        iterate_positions(args, poslist, gd, grid_state, metrology_func=dummy_metrology_func,
                       deviation_list=deviations)
    except:
        print("############# Exception caught ###################")
        printtime()
        gd.pingFPUs(grid_state)
        print("grid state:", grid_state)
        print("FPU state:", grid_state.FPU[0])
        print("positions:", list_angles(grid_state))
        raise
    
    print("counter deviations:", deviations)

    printtime()
    print("ready.")
    
    
if __name__ == '__main__':
    # parse arguments
    args = parse_args()
    rungrid(args)
    
                    

              





