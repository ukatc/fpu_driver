#!/usr/bin/python
from __future__ import print_function, division
"""This module drives one FPU to a fine-grained sequence of 
positions and calls a measurement function at each position.
"""

import sys
import signal
import time
import os
import argparse
import matplotlib
import pylab as pl

from numpy import (array, random, asarray, zeros, ones, sqrt, ceil,
                   cumsum, arange, sin, pi, maximum, sign, append, insert, isnan )

import FpuGridDriver
from FpuGridDriver import (TEST_GATEWAY_ADRESS_LIST, GatewayAddress,
                           SEARCH_CLOCKWISE, SEARCH_ANTI_CLOCKWISE,
                           DEFAULT_WAVEFORM_RULSET_VERSION, DATUM_TIMEOUT_DISABLE)
from fpu_commands import *
from fpu_constants import *

def wf_create(num_fpus=1):
    return { id : [] for id in range(num_fpus)}

def wf_zero():
    return { 0 : [ (0, 0)]}

def wf_copy(wf):
    return {id : list(steps) for id, steps in wf.items() }

def below_minsteps(s, min_steps):
    astep = abs(s)
    return  ((astep > 0 ) and (astep < min_steps))


def filter_slow_tail(step_list, min_steps):
    if len(step_list) == 0:
        return step_list
    #print("before:", step_list)
    asteps, bsteps = zip(*step_list)
    asteps =  list(filter(lambda s: not below_minsteps(s, min_steps), asteps))
    bsteps =  list(filter(lambda s: not below_minsteps(s, min_steps), bsteps))

    #print("asteps:", asteps)
    #print("bsteps:", bsteps)
    while len(asteps) < len(bsteps):
        asteps.append(0)
    while len(bsteps) < len(asteps):
        bsteps.append(0)

    sl = zip(asteps, bsteps)            
    
    #print("after:", sl)
    return sl
    
    

def wf_append(wf1, wf2, min_steps=125, ruleset_version=DEFAULT_WAVEFORM_RULSET_VERSION):
    wf = wf_copy(wf1)
    
    for k in wf.keys():
        if ruleset_version == 2:
            # delete entries which have step numbers
            # smaller than min_steps
            wf[k] = filter_slow_tail(wf[k], min_steps)
                
        if wf2.has_key(k):
            ws = wf2[k]
        else:
            ws = wf2[0]
        wf[k].extend(ws)

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

# returns true if any step indexed with
# idx in any series of the waveform is non-zero
def any_seg_nonzero(wf, idx):
    # iterate over series per FPU
    for ws in wf.values():
        # get indexed step in waveform
        s = ws[idx]
        # check for non-zero
        if  (s[0] != 0) or  (s[1] != 0):
            return True
    return False
        


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
    oscillation_maxsteps = random.randint(0, 1 + opts.start_steps)
    oscillation_max_segments = int(float(opts.max_oscillation_time)/ (opts.segment_length_ms / 1000.0))
    oscillation_segments = random.randint(4, oscillation_max_segments +1)
    oscillation_period_len = 4 * random.randint(1, 1 + oscillation_segments // 4) 

    oscillation_num_periods = oscillation_segments //  oscillation_period_len
    samples = []
    stepped_segs = oscillation_period_len // 4
    steps_per_seg = oscillation_maxsteps // stepped_segs 
    
    for p in range(oscillation_num_periods):
        for sign in (-1,1):
            for k in range(stepped_segs):
                oscillation_steps = random.randint(int(steps_per_seg * 0.8),
                                                   steps_per_seg +1)
                samples.append(sign * oscillation_steps)
                samples.append(0)
            
            

    ws = [ (s, s) for s in samples]
    wf = { j : ws for j in range(len(current_alpha)) }

    return wf

    
    
def gen_creep(current_alpha, current_beta, alpha_target, beta_target,
              opts=None, fraction=0.6, max_change=1.05, speed_factor=0.2):
    dist_alpha = alpha_target - current_alpha
    dist_beta = beta_target - current_beta

    max_steps = opts.start_steps + int( (opts.max_steps - opts.start_steps) * speed_factor)

    wf = gen_wf(dist_alpha * fraction, dist_beta * fraction, max_change=max_change,
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
             

def gen_wait(current_alpha, current_beta, opts=None, nsegments=None, max_segments=None):
    clen = opts.cycle_length

    if nsegments is None:
        nsegments = random.randint(0, max_segments+1)

    series = [ (0, 0) for k in range(nsegments) ]
    wf = { k : series for k in range(len(current_alpha)) }

    return wf


def gen_jump(current_alpha, current_beta, alpha_target, beta_target, opts=None):
    dist_alpha = alpha_target - current_alpha
    dist_beta = beta_target - current_beta

    wf = gen_wf(dist_alpha, dist_beta, max_change=opts.max_acceleration, min_steps=opts.start_steps)

    return wf

def check_wf(wf, channel, opts=None, verbose=False, rulset_version=None):
    for ws in wf.values():
        y = array([ seg[channel] for seg in ws ])
        y = append(y, 0)
        if verbose:
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

            small_y = min(yk_abs, y_prev_abs)
            large_y = max(yk_abs, y_prev_abs)

            if verbose:
                print("v[%i][%i]= %r, small=%i, large=%i, y_post=%i" % (channel, k, yk, small_y, large_y, y_post))

            assert(sign(yk) * sign(y_prev) >= 0)
            assert(sign(yk) * sign(y_post) >= 0)
                   
            if (small_y != 0) and (y_prev != 0) and (y_post != 0):
                assert ( (large_y /  small_y) <= opts.max_acceleration), "step too large for step %i, small=%i, large = %i" % (
                    k, small_y, large_y)
                        


"""generates a waveform according to the given command line parameters."""
def gen_duty_cycle(current_alpha, current_beta, cycle_length=32.0,
                   ruleset_version=DEFAULT_WAVEFORM_RULSET_VERSION,
                   opts=None):

    rest_segments = int(round((1000 * cycle_length) / opts.segment_length_ms))    
    if rest_segments > opts.maxnum_waveform_segments:
        raise ValueError("the time requires more segments than the the number of allowed segments")

    min_steps = opts.min_steps
    
    verbosity = opts.verbosity
    
    wf = wf_create(opts.N)

    if verbosity > 1:
        print("current angles:", current_alpha, current_beta)
    
    smrsix = 0 # that's the state machine rotating state index,
    #            aka SMRSIX
    while True:
        
        if smrsix == 0:
            sname = "fastmove"
            # chose a random final destination for both arms
            alpha_target = random.uniform(opts.alpha_min, opts.alpha_max)
            beta_target = random.uniform(opts.beta_min, opts.beta_max)

            # a fast acceleration 
            wf2 = gen_fastmove(current_alpha, current_beta, alpha_target, beta_target,
                               fraction=0.7, opts=opts)

        elif smrsix == 1:
            sname = "jerks"
            # several jerky movements back and forth - between 1 and 5 jerks, with randomized strength
            njerks = random.randint(0,3)
            wf2 = wf_create()
            wf_z = wf_zero()
            jerk_direction = random.randint(0, 2)
            for k in range(njerks):
                wf_creep = gen_creep(current_alpha, current_beta, alpha_target, beta_target, opts=opts,
                                fraction=0.1, max_change=1.2, speed_factor=0.5)
                wf2 = wf_append(wf_append(wf2, wf_creep, min_steps=min_steps, ruleset_version=ruleset_version),
                                wf_z, min_steps=min_steps, ruleset_version=ruleset_version)
                wf_jerk = gen_jerk(current_alpha, current_beta, jerk_direction, opts=opts)
                jerk_direction = (jerk_direction + 1) % 2
                wf2 = wf_append(wf_append(wf2, wf_jerk, min_steps=min_steps, ruleset_version=ruleset_version),
                                wf_z, min_steps=min_steps, ruleset_version=ruleset_version)
                
        elif smrsix == 2:
            smrsix += 1
            continue
            sname = "oscillations"            
            # a period of small oscillations
            wf_z = wf_zero()
            wf2a = gen_creep(current_alpha, current_beta, alpha_target, beta_target,
                             fraction=0.05, speed_factor=0.3, opts=opts)
            wf2b = gen_oscillation(current_alpha, current_beta, opts=opts)
            wf2 = wf_append(wf_append(wf2a, wf_z, min_steps=min_steps, ruleset_version=ruleset_version),
                            wf2b, min_steps=min_steps, ruleset_version=ruleset_version)
        elif smrsix == 3:
            sname = "slow creep"
                        
            # a slow creeping movement towards the target
            wf2 = gen_creep(current_alpha, current_beta, alpha_target, beta_target,
                            fraction=0.95, opts=opts)
        elif smrsix == 4:
            sname = "wait"

            # a few moments stationary wait - random duration
            wf2 = gen_wait(current_alpha, current_beta, max_segments=int(0.3 * rest_segments), opts=opts)
        elif smrsix == 5:
            sname = "jumptotarget"

            # a short jump to the final destination
            wf2 = gen_jump(current_alpha, current_beta, alpha_target, beta_target, opts=opts)
        else:
            sname = "rest wait"

            # a few moments stationary wait - rest duration
            wf2 = gen_wait(current_alpha, current_beta, nsegments=rest_segments, opts=opts)

        if verbosity > 2:
            print("smrsix", smrsix, "segment:", sname, "wf=", wf2, end="")
            
        wf2_len = len(wf2[0])

        new_alpha = current_alpha + (sum_steps(wf2, "alpha") / StepsPerDegreeAlpha)
        
        new_beta = current_beta + (sum_steps(wf2, "beta") / StepsPerDegreeBeta)
        
        if wf2_len > rest_segments:
            if verbosity > 0:
                print("(discarded, bc length)")
            break
        elif (any(new_alpha < opts.alpha_min)
              or any(new_alpha > opts.alpha_max)
              or any(new_beta < opts.beta_min)
              or any(new_beta > opts.beta_max)):

            if verbosity > 0:
                print("(discarded, bc range)")
            break
        else:
            if verbosity  > 1:
                print(".. ok")
        
        if (wf2_len > 0) and (wf2_len < rest_segments):
            if any_seg_nonzero(wf2, -1):
                wf2 = wf_append(wf2, wf_zero(), min_steps=min_steps, ruleset_version=ruleset_version)
                wf2_len = len(wf2[0])
                
        for channel in [0, 1]:
            check_wf(wf2, channel, opts=opts)
        
        wf = wf_append(wf, wf2, min_steps=min_steps, ruleset_version=ruleset_version)
            
        
        rest_segments -= wf2_len
        
        current_alpha = new_alpha        
        current_beta = new_beta
            
        smrsix = (smrsix + 1) % 6
        if smrsix == 0:
            break
        
    if verbosity > 2:
        print("new angles:", current_alpha, current_beta)
    
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
    
    parser.add_argument('-r', '--ruleset_version',  metavar='RULESET_VERSION', type=int,
                        default=DEFAULT_WAVEFORM_RULSET_VERSION,
                        help="""Version number of rule set which is used for waveform validity checking.
                        Currently available options: 0 - no checking, 1 - strict checking with 
                        small speeds allowed (not fully supported by current firmware)
                        2 - loose checking with small speeds disallowed, as in RFE to Software ICD issue 2.1,
                        and supported by firmware >= 1.4.4. (default: %(default)s)""")
    
    parser.add_argument('--alpha_min', metavar='ALPHA_MIN', type=float, default=ALPHA_MIN_DEGREE,
                        help='minimum alpha value  (default: %(default)s)')
    
    parser.add_argument('--alpha_max', metavar='ALPHA_MAX', type=float, default=ALPHA_MAX_DEGREE,
                        help='maximum alpha value  (default: %(default)s)')
    
    parser.add_argument('--beta_min', metavar='BETA_MIN', type=float, default=BETA_MIN_DEGREE,
                        help='minimum beta value  (default: %(default)s)')
    
    parser.add_argument('--beta_max', metavar='BETA_MAX', type=float, default=BETA_MAX_DEGREE,
                        help='maximum beta value  (default: %(default)s)')

    parser.add_argument('--chill_time', metavar='CHILL_TIME', type=float, default=30,
                        help='chill time between movements  (default: %(default)s)')
        
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

    parser.add_argument('--max_oscillation_time', metavar='MAX_OSCILLATION_TIME', type=float,
                        default=5.0,
                        help='time for small oscillation, in seconds (default: %(default)s)')

    parser.set_defaults(simulate_quantization=False)
    
    parser.add_argument('-Q', '--simulate_quantization', dest="simulate_quantization",  action='store_true',
                        help='simulate quantization of step counts by introducing '
                        'mocked rounding errrors  (default: %(default)s)')

    parser.add_argument('-q', '--no_quantization', dest="simulate_quantization", action='store_false',
                        help='do not simulate quantization of step counts')

            
    parser.add_argument('--end_time', metavar='END_TIME', type=str,
                        default="2025-12-31T12:59:59UTC",
                        help='ISO8601 time stamp of when test ends (default: %(default)s)')

    parser.add_argument('-v', '--verbosity', metavar='VERBOSITY', type=int,
                        default=1,
                        help='verbosity level of progress messages (default: %(default)s)')

     
    args = parser.parse_args()
    args.stop_time = time.strptime(args.end_time, '%Y-%m-%dT%H:%M:%S%Z')

    args.min_steps=int(args.min_step_frequency * args.segment_length_ms / 1000)
    args.max_steps=int(ceil(args.max_step_frequency * args.segment_length_ms / 1000))
    args.start_steps=int(ceil(args.max_start_frequency * args.segment_length_ms / 1000))

    print("""min_steps   = %i
start_steps = %i
max_steps   = %i""" % (args.min_steps, args.start_steps, args.max_steps))
          
    return args


stop_all_fpus = False

def stop_handler(signum, frame):
    global stop_all_fpus
    print("STOPPING LIFE TIME TEST - PLEASE STAND BY")
    stop_all_fpus = True


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
    gd.findDatum(grid_state, timeout=DATUM_TIMEOUT_DISABLE)
    print("findDatum finished")

    # We can use grid_state to display the starting position
    print("the starting position (in degrees) is:", list_angles(grid_state)[0])

        
    signal.signal(signal.SIGQUIT, stop_handler)
    print("press <Ctrl>-'\\' to terminate test orderly, <Ctrl>-c only for emergency abort")

    return gd, grid_state

def chatty_sleep(sleep_time, time_slice=0.2, opts=None):
    n = 0
    while sleep_time > 0:
        slice_len = min(time_slice, sleep_time)

        c = "-\|/"[n % 4]

        if opts.verbosity > 0:
            print("waiting %4.1f sec .... %s\r" % (sleep_time, c), end="")
            sys.stdout.flush()
        time.sleep(slice_len)
        sleep_time -= slice_len
        n += 1
        
    if opts.verbosity > 0:
        print("waiting 0 sec    .... OK")

def out_of_range(ws, channel, current_angle, scale, minval, maxval):
    positions = current_angle  + cumsum([ s[channel] for s in ws]) / scale
    
    if (min(positions) < minval) or (max(positions) > maxval):
        return True
    return False
        

def rungrid(args):
    # initialize FPU accordingly
    verbosity = args.verbosity
    gd, grid_state = initialize_FPU(args)

    deviations = []

    global stop_all_fpus

    try:
        while True:
            t = time.localtime()
            if t > args.stop_time:
                print("end_time = %r, time: %r" % (args.stop_time, t))
                print("Time limit reached: finished.")
                sys.exit(0)
                
            if stop_all_fpus:                
                print("Stop signal received: terminating.")
                sys.exit(0)
                
            # get current angles
            current_angles = gd.countedAngles(grid_state)

            current_alpha = array([x for x, y in current_angles ])
            current_beta = array([y for x, y in current_angles ])
            invalid=True
            while invalid:
                wf = gen_duty_cycle(current_alpha, current_beta, args.cycle_length,
                                    ruleset_version=args.ruleset_version, opts=args)

                invalid=False
                for id, ws in wf.items():
                    if ( out_of_range(ws, 0, current_alpha[id], StepsPerDegreeAlpha, args.alpha_min, args.alpha_max)
                         or out_of_range(ws, 1, current_beta[id], StepsPerDegreeBeta, args.beta_min, args.beta_max)):
                        
                        print("waveform out of range, retry..")
                        invalid=True
                        break
                    
                
                    
            if verbosity > 2:
                for k, ws in enumerate(wf[0]):
                    print("wf[%i] = %r" % (k, ws))

            if verbosity > 0:
                print("configure new waveform")
            if not stop_all_fpus:
                start_time = time.time()
                gd.configMotion(wf, grid_state,
                                ruleset_version=args.ruleset_version)
                finish_time = time.time()
                if verbosity > 0:
                    print("waveform upload duration took %5.2f seconds" % (finish_time - start_time))
            if not stop_all_fpus:
                if verbosity > 0:
                    print("executing waveform")
                gd.executeMotion(grid_state)
                if not stop_all_fpus:
                    chatty_sleep(args.chill_time, opts=args)

                if verbosity > 0:
                    print("reversing waveform")
                gd.reverseMotion(grid_state)
                gd.executeMotion(grid_state)
                
            if not stop_all_fpus:
                chatty_sleep(args.chill_time, opts=args)

            if verbosity > 0:
                print("findDatum")
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
        wf = gen_duty_cycle(current_alpha, current_beta, args.cycle_length,
                            ruleset_version=args.ruleset_version, opts=args)
        plot_waveform(wf, 0)
        plot_steps(wf, 0)


    
                    

              





