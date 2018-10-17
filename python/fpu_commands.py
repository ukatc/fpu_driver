#!/usr/bin/python
from __future__ import division, print_function

import warnings

import math
from numpy import array, asarray, ones_like, ceil, floor, round

""" Utility functions for using the fpu_driver module on the command line.
"""

""" getGridStateSummary takes a gridstate (gs) and returns
    a summary state defined by the lowest commin denominator of each 
    FPUs state."""


from fpu_constants import *

from fpu_driver import getGridStateSummary as gGSS




def list_positions(gs, num_fpus=None, show_zeroed=True):
    """Show positions for each FPU in the grid. The optional second argument
       is the number of FPUs shown."""
    if num_fpus == None:
        num_fpus = len(gs.FPU)
    if show_zeroed:
        return [ (gs.FPU[i].alpha_steps, gs.FPU[i].beta_steps,
                  gs.FPU[i].alpha_was_zeroed, gs.FPU[i].beta_was_zeroed)
                 for i in range(num_fpus)]
    else:
        return [ (gs.FPU[i].alpha_steps, gs.FPU[i].beta_steps)
                 for i in range(num_fpus)]


def list_angles(gs,
                asteps_per_deg=StepsPerDegreeAlpha,
                bsteps_per_deg=StepsPerDegreeBeta,
                show_uninitialized=False,
                alpha_datum_offset=-180.0,
                num_fpus=None):
    """Show approximate angular positions for each FPU in the grid. 
       The optional second and third argument are the scaling factors,
       and the fourth argument is the number of FPUs shown."""
    if num_fpus == None:
        num_fpus = len(gs.FPU)

    nan = float('NaN')
    tvalid = lambda zeroed : 1.0 if (zeroed or show_uninitialized) else nan
    # the following line uses Python3 float division
    return [ ((gs.FPU[i].alpha_steps / asteps_per_deg + alpha_datum_offset)* tvalid(gs.FPU[i].alpha_was_zeroed),
              gs.FPU[i].beta_steps / bsteps_per_deg * tvalid(gs.FPU[i].beta_was_zeroed)) for i in range(num_fpus)]

 
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

def list_serial_numbers(gs, num_fpus=None):
    """List serial number for each FPU in the grid. The optional second argument
       is the number of FPUs shown."""
    if num_fpus == None:
        num_fpus = len(gs.FPU)
    return [ gs.FPU[i].serial_number for i in range(num_fpus)]


# these values are for version 1 firmware and default
# section duration
STEPS_LOWER_LIMIT= int(MOTOR_MIN_STEP_FREQUENCY * WAVEFORM_SEGMENT_LENGTH_MS / 1000)
STEPS_UPPER_LIMIT=int(ceil(MOTOR_MAX_STEP_FREQUENCY * WAVEFORM_SEGMENT_LENGTH_MS / 1000))



def step_list_slow(nsteps, min_steps=STEPS_LOWER_LIMIT,):
    full_segments = int(math.floor(nsteps / min_steps))
    delta = min_steps

    delta_end  = nsteps - delta * full_segments

    slist = [ ]
    
    slist.extend([ delta for k in range(full_segments) ])
    if delta_end > 0:
        slist.append(delta_end)

    return slist

def step_list_fast(nsteps, max_change=1.2,
                   min_steps=STEPS_LOWER_LIMIT, max_steps=STEPS_UPPER_LIMIT):
    
    rest_steps = nsteps
    new_speed = min_steps
    steps_accelerate = []
    steps_decelerate = []
    
    while rest_steps > new_speed:
        steps_accelerate.append(new_speed)
        rest_steps = rest_steps - new_speed
        if rest_steps > new_speed:
            steps_decelerate.append(new_speed)
            rest_steps = rest_steps - new_speed

        old_speed = new_speed
        if int(new_speed * max_change) <= max_steps:
            new_speed = int(new_speed * max_change)
        else:
            new_speed = max_steps
            
        if (new_speed > rest_steps) and (old_speed <= rest_steps):
            new_speed = rest_steps
            
            
    # we handle the remaining steps by padding sections
    # with minimum speed to the start and end.
    # (more elegant solutions are possible).
    
    while rest_steps > (2 * min_steps):
        steps_accelerate.insert(0, min_steps)
        steps_decelerate.insert(0, min_steps)
        rest_steps = rest_steps - 2 * min_steps
        
    if rest_steps > min_steps:
        steps_accelerate.insert(0, min_steps)
        rest_steps = rest_steps -  min_steps

    steps_decelerate.reverse()
    steps_decelerate.append(rest_steps)

    steps_accelerate.extend(steps_decelerate)
    return steps_accelerate


def step_list_pad(slist, target_len):
    slist = list(slist)
    if len(slist) >= target_len:
        return slist
    dlen = target_len - len(slist)

    slist.reverse()
    slist.extend([ 0 ] * dlen)
    slist.reverse()
    return slist


def gen_slist(adegree, bdegree, asteps_per_deg=None, bsteps_per_deg=None,
              mode=None, units="degree", max_change=1.2,
              min_steps=STEPS_LOWER_LIMIT, max_steps=STEPS_UPPER_LIMIT):
    # assert we don't deal with NaNs
    assert( (adegree == adegree) and (bdegree == bdegree))
    # (if the above code confuses you, read https://en.wikipedia.org/wiki/NaN
    # and https://docs.oracle.com/cd/E19957-01/806-3568/ncg_goldberg.html )
    if not (mode in ['fast', 'slow', 'slowpar']):
            raise ValueError("mode needs to be one of 'slow', 'slowpar', 'fast'")

    if units == "degree":
        asteps = int(round(adegree * asteps_per_deg))
        bsteps = int(round(bdegree * bsteps_per_deg))
    elif units == "steps":
        asteps = int(round(adegree))
        bsteps = int(round(bdegree))
    else:
        raise ValueError("The unit needs to be 'steps' or 'degree'")
        
    asign = int(math.copysign(1.0, asteps))
    bsign = int(math.copysign(1.0, bsteps))
    asteps *= asign
    bsteps *= bsign

    if mode in ['slow', 'slowpar']:
        alist = step_list_slow(asteps, min_steps=min_steps)
        blist = step_list_slow(bsteps, min_steps=min_steps)
        max_len = max(len(alist), len(blist))
        alist = step_list_pad(alist, max_len)
        blist = step_list_pad(blist, max_len)
        slist = [ (astep * asign, bstep * bsign)
                  for astep, bstep in zip(alist, blist) ]
    else:
        alist = step_list_fast(asteps, max_change=max_change, min_steps=min_steps, max_steps=max_steps)
        blist = step_list_fast(bsteps, max_change=max_change, min_steps=min_steps, max_steps=max_steps)
        max_len = max(len(alist), len(blist))
        alist = step_list_pad(alist, max_len)
        blist = step_list_pad(blist, max_len)
        
        slist = [ (astep * asign, bstep * bsign)
                  for astep, bstep in zip(alist, blist) ]

    if len(slist) == 0:
        slist = [ (0, 0) ]
    assert(len(slist) <= 128)

    return slist


def gen_wf(aangle, bangle, asteps_per_deg=StepsPerDegreeAlpha,
           bsteps_per_deg=StepsPerDegreeBeta,
           units='degree',
           mode='fast',
           max_change=1.2,
           min_steps=STEPS_LOWER_LIMIT,
           max_steps=STEPS_UPPER_LIMIT):
    """
    Generate a waveform which moves the alpha arm by an angle of 
    adegree and the beta arm by bdegree. asteps_per_deg and bsteps_er_deg
    are approximate calibration factors. The mode parameter can be
    'fast' to generate a movement which is as quick as possible, or 
    'slow' or'slowpar' to generate a slow movement where alpha and beta
    are moved in parallel. (The former 'slow' mode is obsolete,
    it does not match the protocol and capabilities of the current
    firmware).

    If adegree or bdegree are arrays, extend then if possible to a
    common shape, and return a list of waveforms for a number of
    FPUs corresponding to the array.

    No range checking of movements is done.
    """
    if mode == 'slow':
        warnings.warn("'slow' mode is obsolete, it does not match the waveform protocol, mapped to 'slowpar'.")

    aangle = asarray(aangle)
    bangle = asarray(bangle)

    if aangle.ndim == 0:
        aangle.shape = 1

    bangle = ones_like(aangle) * bangle
    aangle = ones_like(bangle) * aangle

    assert(aangle.ndim <= 1)

    slists = dict( (i, gen_slist(aangle[i], bangle[i], asteps_per_deg, bsteps_per_deg,
                                 mode=mode, units=units, max_change=max_change,
                                 min_steps=min_steps, max_steps=max_steps))
                 for i in range(len(aangle)))

    # extend waveform to common maximum length
    maxlen = max(map(len, slists.values()))

    for v in slists.values():
        shortfall = maxlen - len(v)
        if shortfall > 0:
            v.reverse()
            v.extend([(0, 0)] * shortfall)
            v.reverse()
        
    return slists

def path_to_steps(p, steps_per_degree, origin=0.0):
    sum_steps = round((array(p,dtype=float) * RADIAN_TO_DEGREE - origin)
                      * steps_per_degree).astype(int)
    return sum_steps[1:] - sum_steps[:-1], sum_steps
