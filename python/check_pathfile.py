#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function, division


import os
import argparse

from numpy import sign, ceil

from fpu_commands import path_to_steps
from fpu_constants import (RADIAN_TO_DEGREE, MOTOR_MAX_STEP_FREQUENCY, MOTOR_MAX_START_FREQUENCY,
                           MOTOR_MIN_STEP_FREQUENCY, WAVEFORM_SEGMENT_LENGTH_MS,
                           StepsPerDegreeAlpha, StepsPerDegreeBeta, ALPHA_DATUM_OFFSET, BETA_DATUM_OFFSET)
from FpuGridDriver import DEFAULT_WAVEFORM_RULSET_VERSION
from wflib import load_waveform, read_path_file



def parse_args():
    parser = argparse.ArgumentParser(description='check a path file for meeting the waveform validity rules')

    parser.add_argument('pathfile', type=str, nargs=1,
                        default="pgtest/targets_7fp_case_1_1_PATHS.txt",
                        help="""Name of path file""")
    
    parser.add_argument('--show_radian', default=False, action='store_true',
                        help='show angles in radian (the internally used representation)')
    
    parser.add_argument('-r', '--ruleset_version',  metavar='RULESET_VERSION', type=int,
                        default=DEFAULT_WAVEFORM_RULSET_VERSION,
                        help="""Version number of rule set which is used for waveform validity checking.
                        Currently available options: 0 - no checking, 1 - strict checking with 
                        small speeds allowed (not fully supported by current firmware)
                        2 - loose checking with small speeds disallowed, as in RFE to Software ICD issue 2.1,
                        and supported by firmware >= 1.4.4. (default: %(default)s)""")
    
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


class rule:
    label = ""
    description = ""

    @staticmethod
    def check(prev_count, cur_count, next_count, is_last, args):
        return
    
class rule1:
    label = "1"
    description = "no step count must be larger than the max count"

    @staticmethod
    def check(prev_count, cur_count, next_count, is_last, args):
        return abs(cur_count) < args.max_steps


class rule2:
    label = "2"
    description = """except for the last segment, no non-zero \
step count must be smaller than the minimum count"""

    @staticmethod
    def check(prev_count, cur_count, next_count, is_last, args):
        return (cur_count == 0) or is_last or (abs(cur_count) >= args.min_steps)
    
class rule3:
    label = "3"
    description = "no sign reversals are allowed without a zero-step segment between"

    @staticmethod
    def check(prev_count, cur_count, next_count, is_last, args):
        return (sign(prev_count) * sign(cur_count)) >= 0
    
class rule4:
    label = "4"
    description = "No acceleration above the configured threshold is allowed"

    @staticmethod
    def check(prev_count, cur_count, next_count, is_last, args):
        return ((cur_count == 0)
                or (prev_count == 0)
                or (float(cur_count) / prev_count) <= args.max_acceleration )

class rule5:
    label = "5"
    description = "No deceleration above the configured threshold is allowed"

    @staticmethod
    def check(prev_count, cur_count, next_count, is_last, args):
        return ((cur_count == 0)
                or (prev_count == 0)
                or (is_last and (abs(prev_count) <= args.start_steps))
                or (float(prev_count) / cur_count) <= args.max_acceleration )
    
class rule6:
    label = "6"
    description = """if next step is zero and current is not zero, \
speed must be between min speed and start speed"""

    @staticmethod
    def check(prev_count, cur_count, next_count, is_last, args):
        return ((not ((next_count == 0) and (cur_count != 0)))
                or is_last
                or ((abs(cur_count) >= args.min_steps)
                    and (abs(cur_count) <= args.start_steps)))

class rule7:
    label = "7"
    description = """if previous step is zero and current is not zero, \
speed must be between min speed and start speed"""

    @staticmethod
    def check(prev_count, cur_count, next_count, is_last, args):
        return ((not ((prev_count == 0) and (cur_count != 0)))
                or is_last
                or ((abs(cur_count) >= args.min_steps)
                    and (abs(cur_count) <= args.start_steps)))
    
    

class rule2a:
    label = "2a"
    description = """except for the last segment, or if the last step count\
is the minimum count, or if the last step count is zero, no non-zero \
step count must be smaller than the minimum count"""

    @staticmethod
    def check(prev_count, cur_count, next_count, is_last, args):
        return (cur_count == 0) or ( (abs(cur_count) < args.min_steps)
                                     and (is_last
                                          or (prev_count == 0)
                                          or (prev_count == args.min_steps)))

class rule5b:
    label = "5b"
    description = """No deceleration above the square of the configured\
 accelration threshold is allowed"""

    @staticmethod
    def check(prev_count, cur_count, next_count, is_last, args):
        return (cur_count == 0) or (float(prev_count) / cur_count) <= ( args.max_acceleration ** 2)
    


ruleset_v1 = set([rule1, rule2a, rule3, rule4, rule5, rule6, rule7])

ruleset_v2 = set([rule1, rule2, rule3, rule4, rule5, rule6, rule7])

ruleset_v3 = set([rule1, rule2, rule3, rule4, rule5, rule7])


def get_pcn(n, steps):
    assert(n >= 0)
    assert(n < len(steps))
    
    cur = steps[n]
    
    if n > 0:
        prev = steps[n-1]
    else:
        prev = 0
        
    if n < (len(steps) -1):
        next = steps[n+1]
    else:
        next = 0

    return prev, cur, next


def  check_step(nstep, aang, bang,
                alpha_steps, beta_steps,
                asum, bsum,
                ruleset,
                args):
    alpha_failed = set()
    beta_failed = set()

    #print("nstep=", nstep)
    #print("alpha_steps=", alpha_steps)
    

    assert(len(alpha_steps) == len(beta_steps))
    
    
    is_last = (nstep == (len(alpha_steps) -1))

    if not args.show_radian:
        aang *= RADIAN_TO_DEGREE
        bang *= RADIAN_TO_DEGREE
        
    print("%3i: (%8.3f, %8.3f) = Σ_steps (%6i, %6i)  " % (nstep,
                                                          aang, bang,
                                                          asum, bsum), end='')
    if nstep < len(alpha_steps):
        alpha_prev, alpha_cur, alpha_next  = get_pcn(nstep, alpha_steps)
        beta_prev, beta_cur, beta_next  = get_pcn(nstep, beta_steps)
    
        for r in ruleset:
            if not r.check(alpha_prev, alpha_cur, alpha_next, is_last, args):
                alpha_failed.add(r)
                
            if not r.check(beta_prev, beta_cur, beta_next, is_last, args):
                beta_failed.add(r)
                

    
        print(" = steps (%4i,%4i) " % (alpha_cur, beta_cur),
              end='')
        
        if len(alpha_failed) > 0:
            print(", α : %r" % map(lambda r: r.label, alpha_failed), end='')
                    
        if len(beta_failed) > 0:
            print(", β : %r" % map(lambda r: r.label, beta_failed), end='')
                    
    print('')

    return alpha_failed.union(beta_failed)
        
        
                       



if __name__ == '__main__':


    args = parse_args()



    ruleset = { 1 : ruleset_v1,
                2 : ruleset_v2,
                3 : ruleset_v3 }[args.ruleset_version]

    total_failed_rules = set()
    for filename in args.pathfile:
        paths = read_path_file(filename)
        
        # waveform = {}
        for cellid, alpha_path, beta_path  in paths:
            print("=" * 60)
            print("FPU cellid : ", cellid)
            alpha_steps, asum_steps = path_to_steps(alpha_path, StepsPerDegreeAlpha,
                                        origin=ALPHA_DATUM_OFFSET)
            beta_steps, bsum_steps = path_to_steps(beta_path, StepsPerDegreeBeta,
                                       origin=BETA_DATUM_OFFSET)
    
            tseries = zip(alpha_steps, beta_steps)
    
            failed_rules =set()
            for nstep, vals in enumerate(zip(alpha_path, beta_path,                  
                                             asum_steps, bsum_steps)):

                aang, bang, asum, bsum = vals
                
                failed_r = check_step(nstep, aang, bang,
                                      alpha_steps, beta_steps,
                                      asum, bsum,
                                      ruleset,
                                      args)
                failed_rules.update(failed_r)
    
            if len(failed_rules) > 0:
                print("-" * 50, "\n\n")
    
                
                for r in failed_rules:
                    print("%s : %s" % (r.label, r.description))
                print("\n\n\n")
                
            
            total_failed_rules.update(failed_rules)

    print("A total number of %i rules were broken (%s)" % (len(total_failed_rules),
                                                    [x.label for x in total_failed_rules ]))


    


