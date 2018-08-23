#!/usr/bin/python
from __future__ import print_function, division
"""This module drives one FPU to a fine-grained sequence of 
positions and calls a measurement function at each position.
"""

import time
import os
import readline
import argparse

import numpy
from numpy import random, asarray, zeros, ones

import FpuGridDriver
from FpuGridDriver import (TEST_GATEWAY_ADRESS_LIST, GatewayAddress,
                           SEARCH_CLOCKWISE, SEARCH_ANTI_CLOCKWISE)
from fpu_commands import *







def dummy_metrology_func(alpha_steps, beta_steps):
    print("now performing dummy measurement at"
          " alpha={}, beta={} steps".format(alpha_steps,beta_steps))
    ## insert actual measurement stuff here
    time.sleep(1)

def printtime():
    print(time.strftime("%a, %d %b %Y %H:%M:%S +0000 (%Z)", time.gmtime()))
    
def move_fpu(gd, grid_state, alpha_move, beta_move, label=""):

    # generate waveform
    waveform = gen_wf(alpha_move, beta_move)
    
    # configure waveform, by uploading it to the FPU
    print("%s : waveform=%r" %(label, waveform))
            
    gd.configMotion(waveform, grid_state)
    printtime()
    cur_angles = list_angles(grid_state)
    am = asarray(alpha_move) * ones(len(cur_angles))
    bm = asarray(beta_move) * ones(len(cur_angles))
    for i, pos in enumerate(cur_angles):
        a,b = pos
        print("{}: FPU #{}: now moving to ({:6.2f}, {:6.2f})".format(label, i, a + am[i], b + bm[i]))
    gd.executeMotion(grid_state)

    # display the new position
    gd.pingFPUs(grid_state)
    for i, pos in enumerate(list_angles(grid_state)):
        a, b = pos
        print("{}: FPU #{}: reached position: ({:6.2f}, {:6.2f})".format(label, i, a,b) )

    print("timeout count of grid:", grid_state.count_timeout)

    
def measure_position(gd, grid_state, alpha, beta, metrology_func=None,
                     return_to_datum=True, deviation_list=[]):

    #gd.pingFPUs(grid_state)
    positions = asarray(list_angles(grid_state))
    alpha0 = positions[:,0]
    beta0 = positions[:,1]
    for i in range(len(alpha0)):
        print("starting pos: ({:6.2f}, {:6.2f})".format(alpha0[i], beta0[i]))
        
    alpha_move = alpha - alpha0
    beta_move = beta - beta0

    move_fpu(gd, grid_state, alpha_move, beta_move, "out" )
 

    alpha_steps = grid_state.FPU[0].alpha_steps
    beta_steps = grid_state.FPU[0].beta_steps
    # call metrology function for measurement
    metrology_func(alpha_steps, beta_steps)


    if return_to_datum:
        printtime()
        if (alpha0 == 0).all() and (beta0 == 0).all():
            print("now moving back to ({},{})".format(0, 0))
            gd.reverseMotion(grid_state)
            gd.executeMotion(grid_state)
        else:
            move_fpu(gd, grid_state, -alpha, -beta, "return" )

        printtime()
        print("position:", list_angles(grid_state), "steps=", list_positions(grid_state, show_zeroed=False))
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
                        help='reset FPU so that earlier aborts / collisions are ignored (does not work '
                        'with driver 0.7.x because FPU must be initialized before an automatic datum search)')

    parser.add_argument('--gateway_port', metavar='GATEWAY_PORT', type=int, default=4700,
                        help='EtherCAN gateway port number (default: %(default)s)')

    parser.add_argument('--gateway_address', metavar='GATEWAY_ADDRESS', type=str, default="192.168.0.10",
                        help='EtherCAN gateway IP address or hostname (default: %(default)r)')
    
    parser.add_argument('-N', '--NUM_FPUS',  metavar='NUM_FPUS', dest='N', type=int, default=1,
                        help='Number of adressed FPUs. For the deterministic patterns, the FPUs will be steered in unison. For the raodnom patterns, each FPU will receive a random value. WARNING: No conflict checking is done.  (default: %(default)s)')
    
    parser.add_argument('--alpha_min', metavar='ALPHA_MIN', type=float, default=-180.0,
                        help='minimum alpha value  (default: %(default)s)')
    parser.add_argument('--alpha_max', metavar='ALPHA_MAX', type=float, default=172.0,
                        help='maximum alpha value  (default: %(default)s)')
    parser.add_argument('--beta_min', metavar='BETA_MIN', type=float, default=-179.0,
                        help='minimum beta value  (default: %(default)s)')
    parser.add_argument('--beta_max', metavar='BETA_MAX', type=float, default=150.0,
                        help='maximum beta value  (default: %(default)s)')

    parser.add_argument('--chill_time', metavar='CHILL_TIME', type=float, default=1,
                        help='chill time for alpha arm  (default: %(default)s)')

    parser.add_argument('asteps', metavar='ASTEPS', type=int, default=10,
                        help='number of alpha steps  (default: %(default)s)')
    parser.add_argument('bsteps', metavar='BSTEPS', type=int, default=10,
                        help='number of beta steps  (default: %(default)s)')
    
    parser.add_argument('--pattern', metavar='PATTERN', type=str, default="abcircle",
                        choices=['abcircle', 'abc', 'whitenoise', 'wn', 'bluenoise', 'bn', 'polargrid', 'grid'],
                        help="""Pattern of movement. Available patterns are:

                        'abcircle', 'abc' : Calibration pattern as described in VLT-TRE-MON-14620-3007, 
                                     section 5.1.6.
                        'polargrid', 'grid': alpha and beta coordinates are stepped through in a grid 
                                     pattern, with the alpha angle moving after each beta angle
                                     was reached for one alpha position.

                        'whitenoise', 'wn' : In this pattern, a new (alpha, beta) pair of angles 
                                     between the minimum and maximum values is drawn
                                     independently for each step.
                        'bluenoise', 'bn' : In this pattern, new angles are drawn for each step so
                                      that the distribution of distance follows a triangular PDF.

                        default: %(default)r)""")
    
    parser.add_argument('--datum_at', metavar='DATUM_AT', type=str, default="start",
                        choices=['alpha_change', 'beta_change', 'start'],
                        help="go to datum at change of alpha / beta coordinate ('alpha_change', 'beta_change', 'start' default: %(default)r)")
    
    args = parser.parse_args()
    return args



def initialize_FPU(args):
    
    gd = FpuGridDriver.GridDriver(args.N)

    if args.mockup:
        gateway_address = [ FpuGridDriver.GatewayAddress("127.0.0.1", p)
                            for p in [4700, 4701, 4702] ]
    else:
        gateway_address = [ GatewayAddress(args.gateway_address, args.gateway_port) ]

    print("connecting grid:", gd.connect(address_list=gateway_address))


    # We monitor the FPU grid by a variable which is
    # called grid_state, and reflects the state of
    # all FPUs.

    grid_state = gd.getGridState()
    
    gd.pingFPUs(grid_state)
    

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

    ones_vect = numpy.ones(args.N)
    for alpha in numpy.linspace(args.alpha_min, args.alpha_max, args.asteps):
        if args.datum_at == 'alpha_change':
            go_datum = True
        else:
            go_datum = False
            
        for beta in numpy.linspace(args.beta_min, args.beta_max, args.bsteps):
            if args.datum_at == 'beta_change':
                go_datum = True

            yield (alpha * ones_vect, beta * ones_vect, go_datum)        
            go_datum = False

def abcircle_positions(args):

    ones_vect = numpy.ones(args.N)
    # track alpha circle
    beta = min(180, args.beta_max)
    if args.datum_at == 'alpha_change':
        go_datum = True
    else:
        go_datum = False
        
    for alpha in numpy.linspace(args.alpha_min, args.alpha_max, args.asteps):
        yield (alpha, beta, go_datum)        

    # track beta circle
    alpha = max(min(0.0, args.alpha_max), args.alpha_min)
    if args.datum_at == 'beta_change':
        go_datum = True
    else:
        go_datum = False
        
    for beta in numpy.linspace(args.beta_min, args.beta_max, args.bsteps):
        yield (alpha * ones_vect, beta * ones_vect, go_datum)        
            
def whitenoise_positions(args):
    N = args.N
    for j in range(args.asteps):
        alpha = random.uniform(args.alpha_min, args.alpha_max, N)
        if args.datum_at == 'alpha_change':
            go_datum = True
        else:
            go_datum = False

        for j in range(args.bsteps):
            beta = random.uniform(args.beta_min, args.beta_max, N)
            if args.datum_at == 'beta_change':
                go_datum = True

            yield (alpha, beta, go_datum)
            go_datum = False
            

def triangular_distributed(minval, maxval, mode):
    """Computes triangular-distributed values with a vector
    mode parameter.
    """
    # note that the signature of random.triangular and numpy.random.triangular
    # differs in the position of the mode parameter
    return asarray([random.triangular(minval, m, maxval) for m in mode])
            
def bluenoise_positions(args):
    N = args.N
    last_alpha = zeros(N)
    last_beta = zeros(N)
    for j in range(args.asteps):
        alpha = triangular_distributed(args.alpha_min, args.alpha_max, last_alpha)
        if args.datum_at == 'alpha_change':
            go_datum = True
        else:
            go_datum = False

        for j in range(args.bsteps):
            beta = triangular_distributed(args.beta_min, args.beta_max, last_beta)
            if args.datum_at == 'beta_change':
                go_datum = True

            yield (alpha, beta, go_datum)
            last_alpha = alpha
            last_beta = beta


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
    if args.pattern in [ 'polargrid', 'grid']:
        poslist = grid_positions(args)
    elif args.pattern in [ 'abcircle', 'abc']:
        poslist = abcircle_positions(args)
    elif args.pattern in [ 'whitenoise', 'wn']:
        poslist = whitenoise_positions(args)
    elif args.pattern in [ 'bluenoise', 'bn']:
        poslist = bluenoise_positions(args)
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
    
                    

              





