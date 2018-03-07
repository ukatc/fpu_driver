#!/usr/bin/python
from __future__ import division, print_function

import math

""" Utility functions for using the fpu_driver module on the command line.
"""

""" getGridStateSummary takes a gridstate (gs) and returns
    a summary state defined by the lowest commin denominator of each 
    FPUs state."""


from fpu_driver import getGridStateSummary as gGSS


def list_positions(gs, num_fpus=None):
    """Show positions for each FPU in the grid. The optional second argument
       is the number of FPUs shown."""
    if num_fpus == None:
        num_fpus = len(gs.FPU)
    return [ (gs.FPU[i].alpha_steps, gs.FPU[i].beta_steps)
             for i in range(num_fpus)]

def fpu_steps(gs):
    return list_positions(gs)[0]


def list_angles(gs, asteps_per_deg=125, bsteps_per_deg=80, num_fpus=None):
    """Show approximate angular positions for each FPU in the grid. 
       The optional second and third argument are the scaling factors,
       and the fourth argument is the number of FPUs shown."""
    if num_fpus == None:
        num_fpus = len(gs.FPU)

    nan = float('NaN')
    tvalid = lambda fpu : 1.0 if fpu.was_zeroed else nan
    # the following line uses Python3 float division
    return [ (gs.FPU[i].alpha_steps / asteps_per_deg * tvalid(gs.FPU[i]),
              gs.FPU[i].beta_steps / bsteps_per_deg * tvalid(gs.FPU[i])) for i in range(num_fpus)]


def fpu_ang(gs, asteps_per_deg=125, bsteps_per_deg=80,):
    return list_angles(gs, asteps_per_deg=asteps_per_deg,
                       bsteps_per_deg=bsteps_per_deg)[0]

def list_deviations(gs, num_fpus=None):
    """Show datum deviations for each FPU in the grid. The optional second argument
       is the number of FPUs shown."""
    if num_fpus == None:
        num_fpus = len(gs.FPU)
    return [ (gs.FPU[i].alpha_deviation, gs.FPU[i].beta_deviation) for i in range(num_fpus)]

def list_timeouts(gs, num_fpus=None):
    """List counts of timeouts for each FPU in the grid. The optional second argument
       is the number of FPUs shown."""
    if num_fpus == None:
        num_fpus = len(gs.FPU)
    return [ gs.FPU[i].timeout_count for i in range(num_fpus)]

def list_states(gs, num_fpus=None):
    """List state for each FPU in the grid. The optional second argument
       is the number of FPUs shown."""
    if num_fpus == None:
        num_fpus = len(gs.FPU)
    return [ gs.FPU[i].state for i in range(num_fpus)]


# these values are for version 1 firmware and default
# section duration
STEPS_LOWER_LIMIT=125
STEPS_UPPER_LIMIT=500
#STEPS_UPPER_LIMIT=300


def step_list_slow(nsteps):
    full_segments = int(math.floor(nsteps / STEPS_LOWER_LIMIT))
    delta = STEPS_LOWER_LIMIT
    # delta = nsteps // full_segments
    delta_end  = nsteps - delta * full_segments

    slist = [ ]
    
    slist.extend([ delta for k in range(full_segments) ])
    if delta_end > 0:
        slist.append(delta_end)

    return slist

def step_list_fast(nsteps, max_change=1.2):
    rest_steps = nsteps
    new_speed = STEPS_LOWER_LIMIT
    steps_accelerate = []
    steps_decelerate = []
    
    while rest_steps > new_speed:
        steps_accelerate.append(new_speed)
        rest_steps = rest_steps - new_speed
        if rest_steps > new_speed:
            steps_decelerate.append(new_speed)
            rest_steps = rest_steps - new_speed

        old_speed = new_speed
        if int(new_speed * max_change) <= STEPS_UPPER_LIMIT:
            new_speed = int(new_speed * max_change)
        else:
            new_speed = STEPS_UPPER_LIMIT
            
        if (new_speed > rest_steps) and (old_speed <= rest_steps):
            new_speed = rest_steps
            
            
    # we handle the remaining steps by padding sections
    # with minimum speed to the start and end.
    # (more elegant solutions are possible).
    
    while rest_steps > (2 * STEPS_LOWER_LIMIT):
        steps_accelerate.insert(0, STEPS_LOWER_LIMIT)
        steps_decelerate.insert(0, STEPS_LOWER_LIMIT)
        rest_steps = rest_steps - 2 * STEPS_LOWER_LIMIT
        
    if rest_steps > STEPS_LOWER_LIMIT:
        steps_accelerate.insert(0, STEPS_LOWER_LIMIT)
        rest_steps = rest_steps -  STEPS_LOWER_LIMIT

    steps_decelerate.reverse()
    steps_decelerate.append(rest_steps)

    steps_accelerate.extend(steps_decelerate)
    return steps_accelerate


def step_list_pad(slist, target_len):
    if len(slist) >= target_len:
        return slist
    ld = target_len - len(slist)

    lhead = ld // 2
    ltail = ld - lhead
    slist = list(slist)
    slist.reverse()
    slist.extend([ 0] * lhead)
    slist.reverse()
    slist.extend([0] * ltail)
    return slist


def gen_wf(adegree, bdegree, asteps_per_deg=125, bsteps_per_deg=80,
           mode='fast'):
    """
    Generate a waveform which moves the alpha arm by an angle of 
    adegree and the beta arm by bdegree. asteps_per_deg and bsteps_er_deg
    are approximate calibration factors. The mode parameter can be
    'fast' to generate a movement which is as quick as possible, or 
    'slow' to generate a movement with minimum speed, or
    'slowpar' to generate a slow movement where alpha and beta
    are moved in parallel.

    No range checking of movements is done.
    """
    # assert we don't deal with NaNs
    assert( (adegree == adegree) and (bdegree == bdegree))
    # (if the above code confuses you, read https://en.wikipedia.org/wiki/NaN
    # and https://docs.oracle.com/cd/E19957-01/806-3568/ncg_goldberg.html )
    if not (mode in ['fast', 'slow', 'slowpar']):
            raise ValueError("mode needs to be one of 'slow', 'slowpar', 'fast'")
        
    asteps = int(adegree * asteps_per_deg)
    bsteps = int(bdegree * bsteps_per_deg)
    asign = int(math.copysign(1.0, asteps))
    bsign = int(math.copysign(1.0, bsteps))
    asteps *= asign
    bsteps *= bsign

    if mode == 'slow':
        slist = [ (astep * asign, 0) for astep in step_list_slow(asteps) ]
        slist.extend([ (0, bstep * bsign) for bstep in step_list_slow(bsteps) ])
    elif mode == 'slowpar':
        alist = step_list_slow(asteps)
        blist = step_list_slow(bsteps)
        max_len = max(len(alist), len(blist))
        alist = step_list_pad(alist, max_len)
        blist = step_list_pad(blist, max_len)
        slist = [ (astep * asign, bstep * bsign)
                  for astep, bstep in zip(alist, blist) ]
    else:
        alist = step_list_fast(asteps)
        blist = step_list_fast(bsteps)
        max_len = max(len(alist), len(blist))
        alist = step_list_pad(alist, max_len)
        blist = step_list_pad(blist, max_len)
        
        slist = [ (astep * asign, bstep * bsign)
                  for astep, bstep in zip(alist, blist) ]

    if len(slist) == 0:
        slist = [ (0, 0) ]
    assert(len(slist) <= 128)
    return { 0 : slist }
