#!/usr/bin/python
from __future__ import division, print_function

import math
from numpy import asarray, ones_like

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
STEPS_LOWER_LIMIT=125
STEPS_UPPER_LIMIT=500


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


def gen_slist(adegree, bdegree, asteps_per_deg=None, bsteps_per_deg=None,
              mode=None, units="degree"):
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

    return slist


def gen_wf(aangle, bangle, asteps_per_deg=StepsPerDegreeAlpha,
           bsteps_per_deg=StepsPerDegreeBeta,
           units='degree',
           mode='fast'):
    """
    Generate a waveform which moves the alpha arm by an angle of 
    adegree and the beta arm by bdegree. asteps_per_deg and bsteps_er_deg
    are approximate calibration factors. The mode parameter can be
    'fast' to generate a movement which is as quick as possible, or 
    'slow' to generate a movement with minimum speed, or
    'slowpar' to generate a slow movement where alpha and beta
    are moved in parallel.

    If adegree or bdegree are arrays, extend then if possible to a
    common shape, and return a list of waveforms for a number of
    FPUs corresponding to the array.

    No range checking of movements is done.
    """

    aangle = asarray(aangle)
    bangle = asarray(bangle)

    if aangle.ndim == 0:
        aangle.shape = 1

    bangle = ones_like(aangle) * bangle
    aangle = ones_like(bangle) * aangle

    assert(aangle.ndim <= 1)

    slists = dict( (i, gen_slist(aangle[i], bangle[i], asteps_per_deg, bsteps_per_deg,
                                 mode=mode, units=units))
                 for i in range(len(aangle)))

    # extend waveform to common maximum length
    maxlen = max(map(len, slists.values()))

    for v in slists.values():
        v.extend([(0, 0)] * (maxlen - len(v)))
        
    return slists
