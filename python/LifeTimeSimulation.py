#!/usr/bin/python
from __future__ import print_function, division
"""This module drives one FPU to a fine-grained sequence of 
positions and calls a measurement function at each position.
"""

import time
import os
import readline
import argparse
import matplotlib
import pylab as pl
import sys

from numpy import (array, random, asarray, zeros, ones, sqrt, ceil,
                   cumsum, arange, sin, pi, maximum, sign, append, insert )

import FpuGridDriver
from FpuGridDriver import (TEST_GATEWAY_ADRESS_LIST, GatewayAddress,
                           SEARCH_CLOCKWISE, SEARCH_ANTI_CLOCKWISE)
from fpu_commands import *
from fpu_constants import *

def wf_create():
    return { 0 : []}

def wf_zero():
    return { 0 : [ (0, 0)]}

def wf_copy(wf):
    return {id : list(steps) for id, steps in wf.items() }

def wf_append(wf1, wf2):
    wf = wf_copy(wf1)
    
    for k in wf.keys():
        wf[k].extend(wf2[k])

    return wf

def sum_steps(wf, coord):
    if coord == "alpha":
        idx = 0
    elif coord == "beta":
        idx = 1
    else:
        raise ValueError("coord must be 'alpha' or 'beta'")
    
    result = zeros(len(wf), dtype=int)
    for k, val in wf.items():
        result[int(k)] = sum([v[idx] for v in val])
        
    return result

def gen_fastmove(current_alpha, current_beta, alpha_target, beta_target, fraction=0.6, opts=None):
    acceleration = random.uniform(sqrt(opts.max_acceleration), opts.max_acceleration)
    wf2 = gen_wf((alpha_target - current_alpha) * fraction,
                 (beta_target - current_beta) * fraction,
                 max_change=acceleration,
                 min_steps=opts.start_steps)
    return wf2

def gen_jerk(current_alpha, current_beta, jerk_direction, opts=None):
    rel_range = random.uniform(0.05, 0.3)

    if jerk_direction == 0:
        alpha_jmin = current_alpha
        alpha_jmax = current_alpha + (opts.alpha_max - current_alpha) * rel_range
        beta_jmin = current_beta
        beta_jmax = current_beta + (opts.beta_max - current_beta) * rel_range
    else:
        alpha_jmin = current_alpha - (current_alpha - opts.alpha_min) * rel_range
        alpha_jmax = current_alpha
        beta_jmin = current_beta - (current_beta - opts.beta_min) * rel_range
        beta_jmax = current_beta

    new_alpha = random.uniform(alpha_jmin, alpha_jmax)
    new_beta = random.uniform(beta_jmin, beta_jmax)

    wf = gen_wf(new_alpha - current_alpha, new_beta - current_beta,
                max_change=opts.max_acceleration,
                min_steps=opts.start_steps)

    return wf

def gen_oscillation(current_alpha, current_beta, opts=None):
    oscillation_time = random.uniform(1, 5)
    oscillation_segments = int(float(oscillation_time)/ opts.segment_length_ms)
    frq = random.uniform(0.2, 3)
    max_steps = opts.max_steps
    min_steps = opts.min_steps
    max_acc = opts.max_acceleration
    start_steps = opts.start_steps

    oscillation_t = arange(oscillation_segments) * opts.segment_length_ms

    y = max_steps * sin(oscillation_t * 2 * pi * frq)
    # eliminate speed values which are too small


    # filter out values which are too small
    y[abs(y) <  min_steps] = 0
    # set intermediate values to minimum
    y = maximum(abs(y), min_steps) * sign(y)
    # filter out sign jumps
    for k in range(len(y) -1):
        if y[k] * y[k+1] < 0:
            y[k+1] = 0
            
    # smooth out too large accelerations
    for k in range(len(y) -1):
        s = sign(y[k])
        if k == 0:
            prev_y = 0
        else:
            prev_y = y[k-1]
            
        if prev_y == 0:
            if abs(y[k]) > start_steps:
                y[k] = s * random.randint(min_steps, start_steps+1)
            elif abs(y[k]) < min_steps and (y[k] != 0):
                y[k] = s * min_steps
        else:
            if abs(y[k]) > max_acc * abs(prev_y):
                y[k] = s * max_acc * abs(prev_y)

    # smooth out too large decelerations
    # this has the twist that at the end of a movement
    # in one direction, and value between start_steps and zero is allowed
    # once
    k = 0
    while k < len(y):
        s = sign(y[k])
        if k == 0:
            prev_y = 0
        else:
            prev_y = y[k-1]


        if abs(prev_y) > start_steps:
            # normal deceleration
            if abs(y[k]) < abs(prev_y) / max_acc:
                    y[k] = s * abs(prev_y) / max_acc
        else:
            # deceleration when finishing movement streak
            if (y[k] == 0):
                # that's fine, the arm stopped
                pass
            else:
                if abs(y[k]) < abs(prev_y) / max_acc:
                    y[k] = s * abs(prev_y) / max_acc

                # we may need to insert or append a zero value
                if k == (len(y) - 1):
                    # we brake to zero so that the waveform is neutral
                    if abs(y[k]) < min_steps:
                        # below min threhold, next needs to be stop
                        max_next_y = 0
                    else:
                        max_next_y = s * abs(y[k]) / max_acc
                        
                    y = append(y, max_next_y)
                else:
                    if abs(y[k]) < min_steps:
                        # we need to ensure that the next y is zero
                        next_y = y[k+1]
                        if abs(next_y) > 0:
                            y = insert(y, k+1, 0)
                        
                    
                    

        k += 1
        

    series = [ (yt, yt) for yt in y ]
    wf = { j : series for j in range(len(current_alpha)) }

    return wf

    
    
def gen_creep(current_alpha, current_beta, alpha_target, beta_target,
              opts=None, fraction=0.6, max_change=1.01, speed_factor=0.2):
    dist_alpha = alpha_target - current_alpha
    dist_beta = beta_target - current_beta

    max_steps = opts.start_steps + int( (opts.max_steps - opts.start_steps) * speed_factor)

    wf = gen_wf(dist_alpha * fraction, dist_beta * fraction, max_change,
                min_steps=opts.start_steps, max_steps=max_steps)

    # inject a few low step counts, to simulate rounding effects
    if opts.simulate_quantization:
         for ws in wf.values():
            for k in range(len(ws)):
                if random.randint(5) != 0:
                    continue
                segment = list(ws[k])
                for c in [0,1]:
                    if segment[c] <= opts.start_steps:
                        segment[c] = random.randint(opts.min_steps, opts.start_steps+1)
                ws[k]=tuple(segment)
                        
        
    return wf
             

def gen_wait(current_alpha, current_beta, opts=None, nsegments=None):
    clen = opts.cycle_length

    if nsegments is None:
        wait_time = random.uniform(0.05 * clen, 0.1 * clen)
        nsegments = int(round(1000 * wait_time / opts.segment_length_ms))

    series = [ (0, 0) for k in range(nsegments) ]
    wf = { k : series for k in range(len(current_alpha)) }

    return wf


def gen_jump(current_alpha, current_beta, alpha_target, beta_target, opts=None):
    dist_alpha = alpha_target - current_alpha
    dist_beta = beta_target - current_beta

    wf = gen_wf(dist_alpha, dist_beta, max_change=opts.max_acceleration, min_steps=opts.start_steps)

    return wf

def check_wf(wf, channel, opts):
    for ws in wf.values():
        y = array([ seg[channel] for seg in ws ])
        y = append(y, 0)
        print("y=", y)
        len_y = len(y)
        for k, yk in enumerate(y):
            if k == 0:
                y_prev = 0
            else:
                y_prev = y[k-1]
            if k == len_y -1:
                y_post = 0
            else:
                y_post = y[k+1]
            
                
            yk_abs = abs(yk)
            y_prev_abs = abs(y_prev)
            y_post_abs = abs(y_post)
            
            large_y = max(yk_abs, y_prev_abs)
            small_y = min(yk_abs, y_prev_abs)
            
            print("v[%i][%i]= %r, small=%i, large=%i, y_post=%i" % (channel, k, yk, small_y, large_y, y_post))

            assert(sign(yk) * sign(y_prev) >= 0)
            assert(sign(yk) * sign(y_post) >= 0)
                   
            if (small_y != 0) and (y_prev != 0) and (y_post != 0):
                assert( (large_y /  small_y) <= opts.max_acceleration)
                        


"""generates a waveform according to the given command line parameters."""
def gen_duty_cycle(current_alpha, current_beta, cycle_length=32.0, opts=None):

    rest_segments = int(round((1000 * cycle_length) / opts.segment_length_ms))    
    if rest_segments > opts.maxnum_waveform_segments:
        raise ValueError("the time requires more segments than the the number of allowed segments")
    
    wf = wf_create()

    
    smrsix = 0 # that's the state machine rotating state index
    while True:
        
        if smrsix == 0:
            # chose a random final destination for both arms
            alpha_target = random.uniform(opts.alpha_min, opts.alpha_max)
            beta_target = random.uniform(opts.beta_min, opts.beta_max)

            # a fast acceleration 
            wf2 = gen_fastmove(current_alpha, current_beta, alpha_target, beta_target,
                               fraction=0.7, opts=opts)

        elif smrsix == 1:
            # several jerky movements back and forth - between 1 and 5 jerks, with randomized strength
            njerks = random.randint(1,6)
            wf2 = wf_create()
            wf_z = wf_zero()
            jerk_direction = random.randint(0, 2)
            for k in range(njerks):
                wf_creep = gen_creep(current_alpha, current_beta, alpha_target, beta_target, opts=opts,
                                fraction=0.1, max_change=1.2, speed_factor=0.5)
                wf2 = wf_append(wf_append(wf2, wf_creep), wf_z)
                wf_jerk = gen_jerk(current_alpha, current_beta, jerk_direction, opts=opts)
                jerk_direction = (jerk_direction + 1) % 2
                wf2 = wf_append(wf_append(wf2, wf_jerk), wf_z)
                
        elif smrsix == 2:
            # a period of small oscillations
            wf2a = gen_creep(current_alpha, current_beta, alpha_target, beta_target,
                             fraction=0.05, speed_factor=0.3, opts=opts)
            wf2b = gen_oscillation(current_alpha, current_beta, opts=opts)
            wf2 = wf_append(wf2a, wf2b)
        elif smrsix == 3:
            # a slow creeping movement towards the target
            wf2 = gen_creep(current_alpha, current_beta, alpha_target, beta_target,
                            fraction=0.95, opts=opts)
        elif smrsix == 4:
            # a few moments stationary wait - random duration
            wf2 = gen_wait(current_alpha, current_beta, opts=opts)
        elif smrsix == 5:
            # a short jump to the final destination
            wf2 = gen_jump(current_alpha, current_beta, alpha_target, beta_target, opts=opts)
        else:
            # a few moments stationary wait - rest duration
            nsegments = rest_segments
            wf2 = gen_wait(current_alpha, current_beta, nsegments, opts=opts)

        wf2_len = len(wf2[0])
        if wf2_len <= rest_segments:
            print("smrsix=", smrsix)

            for channel in [0, 1]:
                check_wf(wf2, channel, opts)
                
            
            wf = wf_append(wf, wf2)
            smrsix = (smrsix + 1) % 6
            
            rest_segments -= wf2_len
            current_alpha += (sum_steps(wf2, "alpha") / StepsPerDegreeAlpha)
            current_beta += (sum_steps(wf2, "beta") / StepsPerDegreeBeta)
        else:
            break
            

    return wf



def printtime():
    print(time.strftime("%a, %d %b %Y %H:%M:%S +0000 (%Z)", time.gmtime()))
    

      


def parse_args():
    parser = argparse.ArgumentParser(description='Drive FPU to a grid of positions and call measurement function')

    parser.add_argument('command', type=str, nargs='?',
                        default="drive",
                        help="""command word:
                        drive - run waveform against grid until time expires
                        plot - plot one waveform""")
    
    parser.add_argument('--mockup',   default=False, action='store_true',
                        help='set gateway address to use mock-up gateway and FPU')

    parser.add_argument('--resetFPUs',   default=False, action='store_true',
                        help='reset FPUs at start')

    parser.add_argument('--rewind_fpus', default=False, action='store_true',
                        help='rewind FPUs to datum position at start')

    parser.add_argument('--gateway_port', metavar='GATEWAY_PORT', type=int, default=4700,
                        help='EtherCAN gateway port number (default: %(default)s)')

    parser.add_argument('--gateway_addresses', metavar='GATEWAY_ADDRESSES', type=str, default="192.168.0.10",
                        help="""EtherCAN gateway IP addresses or hostnames, 
                                separated by ',' (default: %(default)r)""")
    
    parser.add_argument('-N', '--NUM_FPUS',  metavar='NUM_FPUS', dest='N', type=int, default=7,
                        help="""Number of adressed FPUs. The FPUs will be steered in unison.
                                 WARNING: No conflict checking is done.  (default: %(default)s)""")
    
    parser.add_argument('--alpha_min', metavar='ALPHA_MIN', type=float, default=ALPHA_MIN_DEGREE,
                        help='minimum alpha value  (default: %(default)s)')
    
    parser.add_argument('--alpha_max', metavar='ALPHA_MAX', type=float, default=ALPHA_MAX_DEGREE,
                        help='maximum alpha value  (default: %(default)s)')
    
    parser.add_argument('--beta_min', metavar='BETA_MIN', type=float, default=BETA_MIN_DEGREE,
                        help='minimum beta value  (default: %(default)s)')
    
    parser.add_argument('--beta_max', metavar='BETA_MAX', type=float, default=BETA_MAX_DEGREE,
                        help='maximum beta value  (default: %(default)s)')

    parser.add_argument('--chill_time', metavar='CHILL_TIME', type=float, default=30,
                        help='chill time for alpha arm  (default: %(default)s)')
        
    parser.add_argument('--cycle_length', metavar='CYCLE_LENGTH', type=float, default=32,
                        help='cycle duration  (default: %(default)s)')
        

    parser.add_argument('--min_step_frequency', metavar='min_step_frequency', type=float,
                       default=MOTOR_MIN_STEP_FREQUENCY,
                        help='minimum motor step frequency  (default: %(default)s)')
        
    parser.add_argument('--max_step_frequency', metavar='max_step_frequency', type=float,
                       default=MOTOR_MAX_STEP_FREQUENCY,
                        help='maximum motor step frequency  (default: %(default)s)')
        
    parser.add_argument('--max_start_frequency', metavar='max_start_frequency', type=float,
                        default=MOTOR_MAX_START_FREQUENCY,
                        help='maximum motor start frequency  (default: %(default)s)')
        
        
    parser.add_argument('--max_acceleration', metavar='MAX_ACCELERATION', type=float,
                        default=MAX_ACCELERATION_FACTOR,
                        help='maximum motor acceleration  (default: %(default)s)')

    parser.add_argument('--segment_length_ms', metavar='SEGMENT_LENGTH_MS', type=float,
                        default=WAVEFORM_SEGMENT_LENGTH_MS,
                        help='waveform segment length, in milliseconds (default: %(default)s)')

    parser.add_argument('--maxnum_waveform_segments', metavar='MAXNUM_WAVEFORM_SEGMENTS', type=float,
                        default=MAXNUM_WAVEFORM_SEGMENTS,
                        help='waveform segment length, in milliseconds (default: %(default)s)')

    parser.set_defaults(simulate_quantization=False)
    
    parser.add_argument('-Q', '--simulate_quantization', dest="simulate_quantization",  action='store_true',
                        help='simulate quantization of step counts by introducing '
                        'mocked rounding errrors  (default: %(default)s)')

    parser.add_argument('-q', '--no_quantization', dest="simulate_quantization", action='store_false',
                        help='do not simulate quantization of step counts')

            
    parser.add_argument('--end_time', metavar='END_TIME', type=str,
                        default="2025-12-31T12:59:59UTC",
                        help='ISO8601 time stamp of when test ends (default: %(default)s)')

     
    args = parser.parse_args()
    args.stop_time = time.strptime(args.end_time, '%Y-%m-%dT%H:%M:%S%Z')

    args.min_steps=int(args.min_step_frequency * args.segment_length_ms / 1000)
    args.max_steps=int(ceil(args.max_step_frequency * args.segment_length_ms / 1000))
    args.start_steps=int(ceil(args.max_start_frequency * args.segment_length_ms / 1000))

    print("""min_steps   = %i
start_steps = %i
max_steps   = %i""" % (args.min_steps, args.start_steps, args.max_steps))
          
    return args



def initialize_FPU(args):
    
    gd = FpuGridDriver.GridDriver(args.N)

    if args.mockup:
        gateway_address = [ FpuGridDriver.GatewayAddress("127.0.0.1", p)
                            for p in [4700, 4701, 4702] ]
    else:
        gateway_address = [ GatewayAddress(adr, args.gateway_port)
                            for adr in args.gateway_addresses.split(",")]

    print("connecting grid:", gd.connect(address_list=gateway_address))


    # We monitor the FPU grid by a variable which is
    # called grid_state, and reflects the state of
    # all FPUs.

    grid_state = gd.getGridState()
    
    gd.pingFPUs(grid_state)
    

    if args.resetFPUs:
        print("resetting FPUs")
        gd.resetFPUs(grid_state)
        print("OK")

    # Now, we issue a findDatum method. In order to know when and how
    # this command finished, we pass the grid_state variable.

    if args.rewind_fpus:
        current_angles = gd.trackedAngles(grid_state, retrieve=True)
        current_alpha = array([x.as_scalar() for x, y in current_angles ])
        current_beta = array([y.as_scalar() for x, y in current_angles ])
        print("current positions:\nalpha=%r,\nbeta=%r" % (current_alpha, current_beta))
              
        if False :
              
            print("moving close to datum switch")
            wf = gen_wf(- current_alpha + 0.5, - current_beta + 0.5)
            gd.configMotion(wf, grid_state, allow_uninitialized=True)
            gd.executeMotion(grid_state)
    
    
    print("issuing findDatum:")
    gd.findDatum(grid_state)
    print("findDatum finished")

    # We can use grid_state to display the starting position
    print("the starting position (in degrees) is:", list_angles(grid_state)[0])

    return gd, grid_state


    

def rungrid(args):
    # initialize FPU accordingly
    gd, grid_state = initialize_FPU(args)

    deviations = []

    try:
        while True:
            t = time.gmtime()
            print("end_time = %r, time: %r" % (args.stop_time, t))
            if t > args.stop_time:
                print("Time limit reached: finished.")
                sys.exit(0)
                
            # get current angles
            current_angles = gd.countedAngles(grid_state)
            current_alpha = array([x for x, y in current_angles ])
            current_beta = array([y for x, y in current_angles ])
            wf = gen_duty_cycle(current_alpha, current_beta, args.cycle_length, args)
            gd.configMotion(wf, grid_state)
            gd.executeMotion(grid_state)
            time.sleep(args.chill_time)
            gd.reverseMotion(grid_state)
            gd.executeMotion(grid_state)
            time.sleep(args.chill_time)
            gd.findDatum(grid_state)
    
    except SystemExit:
        pass
    except:
        print("############# Exception caught ###################")
        printtime()
        gd.pingFPUs(grid_state)
        print("grid state:", grid_state)
        for fpu_id, fpu in enumerate(grid_state.FPU):
            if fpu.last_status != 0:
                print("FPU %i state:" % fpu_id, fpu)
        print("positions:", gd.countedAngles(grid_state))
        raise
    
    printtime()
    print("ready.")
    
def plot_waveform(wf, idx):
    steps = array(wf[0]).T
    
    t_sec = arange(steps.shape[1]) * args.segment_length_ms * 0.001
        
    alpha_degree = cumsum(steps[0]) / StepsPerDegreeAlpha
    beta_degree = cumsum(steps[1]) / StepsPerDegreeBeta
        
    h1, = pl.plot(t_sec, alpha_degree, '-', label='alpha over time')
        
    pl.hold(True)
            
    h2, = pl.plot(t_sec, beta_degree, '-', label='beta over time')
        
    pl.legend(handles=[h1, h2])
    pl.xlabel("time  [sec]")
    pl.ylabel("angle [degree]")
    
    pl.show()

    
def plot_steps(wf, idx):
    steps = array(wf[0]).T
    
    t_sec = arange(steps.shape[1]) * args.segment_length_ms * 0.001
        
    alpha_steps = steps[0]
    beta_steps =  steps[1]
        
    h1, = pl.plot(t_sec, alpha_steps, '.', label='alpha over time')
        
    pl.hold(True)
            
    h2, = pl.plot(t_sec, beta_steps, '.', label='beta over time')
        
    pl.legend(handles=[h1, h2])
    pl.xlabel("time  [sec]")
    pl.ylabel("steps [1]")
    
    pl.show()

    
if __name__ == '__main__':
    # parse arguments
    args = parse_args()
    print("command=",args.command)
    
    if args.command == "drive":
        rungrid(args)
    elif args.command == "plot":
        
        current_alpha=0
        current_beta=0
        wf = gen_duty_cycle(current_alpha, current_beta, args.cycle_length, args)
        plot_waveform(wf, 0)
        plot_steps(wf, 0)


    
                    

              





