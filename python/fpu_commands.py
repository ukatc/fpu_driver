#!/usr/bin/python
from __future__ import division, print_function

import warnings

import math
import bisect
from numpy import array, asarray, ones_like, ceil, floor, round

""" Utility functions for using the fpu_driver module on the command line.
"""

""" getGridStateSummary takes a gridState (gs) and returns
    a summary state defined by the lowest common denominator of each
    FPUs state."""


from fpu_constants import *

from ethercanif import getGridStateSummary as gGSS




def list_positions(gs, num_fpus=None, show_zeroed=True):
    """
    
    Show positions for each FPU in the grid. The optional second argument
    is the number of FPUs shown.
    
    """
    if num_fpus == None:
        num_fpus = len(gs.FPU)
    if show_zeroed:
        return [ (gs.FPU[i].alpha_steps, gs.FPU[i].beta_steps,
                  gs.FPU[i].alpha_was_referenced, gs.FPU[i].beta_was_referenced)
                 for i in range(num_fpus)]
    else:
        return [ (gs.FPU[i].alpha_steps, gs.FPU[i].beta_steps)
                 for i in range(num_fpus)]


def list_angles(gs,
                asteps_per_deg=StepsPerDegreeAlpha,
                bsteps_per_deg=StepsPerDegreeBeta,
                show_uninitialized=False,
                alpha_datum_offset=-180.0,
                beta_datum_offset=-6.5,
                num_fpus=None):
    """
    
    Show approximate angular positions for each FPU in the grid.
    The optional second and third argument are the scaling factors,
    and the fourth argument is the number of FPUs shown.
       
    """
    if num_fpus == None:
        num_fpus = len(gs.FPU)

    nan = float('NaN')
    tvalid = lambda zeroed : 1.0 if (zeroed or show_uninitialized) else nan
    # the following line uses Python3 float division
    return [ ((gs.FPU[i].alpha_steps / asteps_per_deg + alpha_datum_offset) * tvalid(gs.FPU[i].alpha_was_referenced),
              (gs.FPU[i].beta_steps / bsteps_per_deg + beta_datum_offset) * tvalid(gs.FPU[i].beta_was_referenced)) for i in range(num_fpus)]


def list_deviations(gs, num_fpus=None):
    """
    
    Show datum deviations for each FPU in the grid. The optional second argument
    is the number of FPUs shown.
    
    """
    if num_fpus == None:
        num_fpus = len(gs.FPU)
    return [ (gs.FPU[i].alpha_deviation, gs.FPU[i].beta_deviation) for i in range(num_fpus)]

def list_timeouts(gs, num_fpus=None):
    """
    
       List counts of timeouts for each FPU in the grid. The optional second argument
       is the number of FPUs shown.
    
    """
    if num_fpus == None:
        num_fpus = len(gs.FPU)
    return [ gs.FPU[i].timeout_count for i in range(num_fpus)]

def list_states(gs, num_fpus=None):
    """
    
       List state for each FPU in the grid. The optional second argument
       is the number of FPUs shown.
    
    """
    if num_fpus == None:
        num_fpus = len(gs.FPU)
    return [ gs.FPU[i].state for i in range(num_fpus)]

def list_serial_numbers(gs, num_fpus=None):
    """
    
       List serial number for each FPU in the grid. The optional second argument
       is the number of FPUs shown.
    
    """
    if num_fpus == None:
        num_fpus = len(gs.FPU)
    return [ gs.FPU[i].serial_number for i in range(num_fpus)]



# these values are for version 1 firmware and default
# section duration
STEPS_LOWER_LIMIT= int(MOTOR_MIN_STEP_FREQUENCY * WAVEFORM_SEGMENT_LENGTH_MS / 1000)
STEPS_UPPER_LIMIT=int(ceil(MOTOR_MAX_STEP_FREQUENCY * WAVEFORM_SEGMENT_LENGTH_MS / 1000))



def step_list_slow(nsteps, min_steps=STEPS_LOWER_LIMIT,):

    """
    
       Generate alpha or beta angles in slow mode.
    
       *** THIS FUNCTION IS NO LONGER USED ***
       
    """

    full_segments = int(math.floor(nsteps / min_steps))
    delta = min_steps

    delta_end  = nsteps - delta * full_segments

    slist = [ ]

    slist.extend([ delta for k in range(full_segments) ])
    if delta_end > 0:
        slist.append(delta_end)

    # Check the step list achieves the target
    (steptotal, maxchange) = step_list_count(slist)
    if ( steptotal != nsteps ):
        strg = "step_list_slow: Failed to achieve target: %d steps planned, %s steps actual." % \
               (nsteps, steptotal)
        strg += "\n\tPlease raise a bug ticket."
        raise ValueError(strg)

    return slist

def step_list_fast(nsteps, max_change=1.4,
                   min_steps=STEPS_LOWER_LIMIT, max_steps=STEPS_UPPER_LIMIT):

    """
    
       Generate alpha or beta angles in fast mode.
       This function implements an old waveform generation mode. Whether it
       is faster than linacc mode depends on the values for the max_change
       and max_acceleration parameters.
    
       *** OBSOLETE FUNCTION - NO LONGER NEEDED ***
    
    """

    remaining_steps = nsteps
    new_speed = min_steps
    steps_accelerate = []
    steps_decelerate = []

    while remaining_steps > new_speed:
        steps_accelerate.append(new_speed)
        remaining_steps = remaining_steps - new_speed
        if remaining_steps > new_speed:
            steps_decelerate.append(new_speed)
            remaining_steps = remaining_steps - new_speed

        old_speed = new_speed
        if int(new_speed * max_change) <= max_steps:
            new_speed = int(new_speed * max_change)
        else:
            new_speed = max_steps

        if (new_speed > remaining_steps) and (old_speed <= remaining_steps):
            new_speed = remaining_steps


    # we handle the remaining steps by padding sections
    # with minimum speed to the start and end.
    # (more elegant solutions are possible).

    while remaining_steps > (2 * min_steps):
        steps_accelerate.insert(0, min_steps)
        steps_decelerate.insert(0, min_steps)
        remaining_steps = remaining_steps - 2 * min_steps

    if remaining_steps > min_steps:
        steps_accelerate.insert(0, min_steps)
        remaining_steps = remaining_steps -  min_steps

    steps_decelerate.reverse()
    steps_decelerate.append(remaining_steps)

    steps_accelerate.extend(steps_decelerate)

    # Check the step list achieves the target
    (steptotal, maxchange) = step_list_count(steps_accelerate)
    if ( steptotal != nsteps ):
        strg = "step_list_fast: Failed to achieve target: %d steps planned, %s steps actual." % \
               (nsteps, steptotal)
        strg += "\n\tPlease raise a bug ticket."
        raise ValueError(strg)

    return steps_accelerate


def step_list_limacc(nsteps,                                  # Total number of steps to move
                     max_acceleration=MOTOR_MAX_ACCELERATION, # Max acceleration
                     max_deceleration=MOTOR_MAX_DECELERATION, # Max deceleration
                     min_steps=STEPS_LOWER_LIMIT,             # Minimum speed
                     min_stop_steps=STEPS_LOWER_LIMIT,        # Minimum stop speed
                     max_steps=STEPS_UPPER_LIMIT,             # Maximum speed
                     insert_rest_accelerate=None):            # *** NO LONGER NEEDED: Where to pad the waveform

    """
    
       Generate alpha or beta angles in limacc (constant acceleration) mode.

       The function constructs an acceleration waveform and a deceleration
       waveform and joins them back to back to make the complete profile.

       A waveform consists of: (a) an acceleration phase; (b) a drift at
       constant speed phase; (c) a deceleration phase; (d) a creep phase
       to move the remaining steps at minimum speed. Only long waveforms
       will have all 4 phases. There is a critical distance below which
       the drift phase disappears and the acceleration and deceleration
       phases join back-to-back. Very short waveforms will only have a
       creep phase.

       TODO: SMB Can this function be simplified? The lengths of the phases
       ought to be predictable, rather than having them built in while loops
       with specific exit conditions.
       
       *** THIS FUNCTION IS NEEDED BUT OVER COMPLICATED. SIMPLIFY. ***
       
       *** Max acceleration is always the same as max deceleration.
       *** insert_rest_accelerate is always None. Other options not needed.

    """
    # Initialise the step lists for the acceleration and deceleration phases
    # and remaining step count.
    remaining_steps = nsteps
    steps_accelerate = []
    steps_decelerate = []

    if min_stop_steps is None:
        min_stop_steps = min_steps

#    print("step_list_limacc nsteps=%d: min_steps=%r, min_stop_steps=%r, max_steps=%r, max_acceleration=%r, max_deceleration=%r" % \
#          (nsteps, min_steps, min_stop_steps, max_steps, max_acceleration, max_deceleration ))

    # Initialise the available range of speed for the current waveform element.
    # Note that each variable is a 2-element array where element [0] is
    # constructed for the acceleration phase and element [1] is constructed
    # for the deceleration phase.
    # steps[0] contains the step elements for the acceleration phase and
    # steps[1] contain the step elements for the deceleration phase.
    new_speed = [min_steps, min_stop_steps]
    max_change = [max_acceleration, max_deceleration]
    max_speed = [min_steps + max_acceleration, min_stop_steps + max_deceleration]
    steps = [[],[]]

    # These constants identify the acceleration [0] and deceleration [1] phases
    # for the above arrays. Note that the variable W indicates which phase is
    # being worked on at any moment and the variable X indicates the phase which
    # isn't being worked on. The function will work alternately on the acceleration
    # and then the deceleration phase until the maximum speed is reached or
    # the motor moves close to its target.
    ACC = 0
    DEC = 1

    # Construct the major part of the waveform
    while True:
        if remaining_steps <= 2*min_stop_steps:
            # The waveform brings the motor within two elements of its target.
            # No further acceleration or deceleration can be added.
#            print("Within two elements (remaining:%d<=%d) No further acceleration/deceleration possible." % \
#                 (remaining_steps, 2*min_stop_steps))
            break

        if len(steps[ACC]) == 0:
            # First iteration - start by working on the acceleration phase.
            W = ACC
            X = DEC
#            print("First iteration. Acceleration phase.")
        elif len(steps[DEC]) == 0:
            # Second interation - now start working on the deceleration phase.
            W = DEC
            X = ACC
#            print("First iteration. Deceleration phase.")
        elif new_speed[ACC] <= new_speed[DEC]:
            # The acceleration phase is behind the deceleration phase
            # (or we have reached a constant speed phase).
            # Work on the acceleration phase.
            W = ACC
            X = DEC
#            print("Working on acceleration phase.")
        else:
            # The deceleration phase is behind the acceleration phase.
            # Work on the deceleration phase.
            W = DEC
            X = ACC
#            print("Working on deceleration phase.")

        if  new_speed[W] > remaining_steps - min_steps:
            # We can't accelerate more.
#            print("New speed (%d) > remaining-min (%d). No further acceleration." % \
#                  (new_speed[W], remaining_steps-min_steps))
            break

        # compute new speed from acceleration limit.
        # (tent_new_speed means "tentative new speed")
        if len(steps[W]) == 0:
            if W == ACC:
                # Starting from stationary (working forwards on acceleration phase)
                tent_new_speed = min_steps
            else:
                # Stopping (working backwards on deceleration phase)
                tent_new_speed = min_stop_steps
        else:
            # Attempt to accelerate or decelerate.
            tent_new_speed = new_speed[W] + max_change[W]

#        print("Tentative new speed=", tent_new_speed, "while remaining steps=", remaining_steps)

        # Check for max speed limit
        if tent_new_speed > max_steps:
#            print("Clip at speed limit:", max_steps)
            tent_new_speed = max_steps

        # If the speed exhausts available distance (to within the minimum step count), cap the
        # speed to the smaller of remaining the distance, or counter-acting change.
#        print("counter-acting change, max_speed[X]=", max_speed[X])
        if (tent_new_speed + max_change[X] > remaining_steps) :
#            print("Clip by counter-acting change:",  max_speed[X])
            tent_new_speed = min(tent_new_speed, max_speed[X])

        # The new speed must leave at least min_steps remaining.
        tent_new_speed = min(tent_new_speed, remaining_steps - min_steps)
#        print("Clip by steps remaining:", tent_new_speed)

        # After applying the limits, accept the new speed step.
        new_speed[W] = tent_new_speed
        steps[W].append(new_speed[W])
        max_speed[W] = tent_new_speed + max_change[W]
        remaining_steps -= new_speed[W]
#        print("Remaining steps becomes:", remaining_steps)

    # At this point the acceleration and deceleration phases have
    # been constructed. What remains is to assemble the waveform
    # and adjust it so its length corresponds exactly to the
    # requested number of steps.

    # If insert_rest_accelerate=True, insert extra steps into acceleration phase
    # If insert_rest_accelerate=False, insert extra steps into deceleration phase
    # If insert_rest_accelerate=None, add extra steps at end of waveform
    if insert_rest_accelerate is not None:
        if insert_rest_accelerate:
            # Adjust the waveform by inserting a rest
            # into the acceleration phase.
            W = ACC
        else:
            # Adjust the waveform by inserting a rest
            # into the deceleration phase.
            W = DEC

        # Set the default maximum speed used for
        # very small movements.
        if W == ACC:
            max_speed = min_steps
        else:
            max_speed = min_stop_steps

        # If there are elements within the working array
        # set the maximum speed the last defined speed.
        if len(steps[W]) > 0:
            max_speed = steps[W][-1]

        # Find a suitable point within the step list in
        # which to insert the rest
        while remaining_steps >= min_steps:
            ins_steps = min(remaining_steps, max_speed)
#            print("Inserting", ins_steps, "steps into waveform")
            bisect.insort(steps[W], ins_steps)
            remaining_steps -= ins_steps

    # Construct the complete waveform by joining the
    # acceleration and deceleration phases back to back.
    steps_accelerate = steps[ACC]
    steps_decelerate = steps[DEC]
    steps_decelerate.reverse()

    # If there are any residual steps remaining, append them to the end
    # of the deceleration phase as a constant drift at minimum speed.
    # The last element is allowed to contain an element smaller than
    # the minimum speed.
    if remaining_steps > 0:
#       print("There are", remaining_steps, "residual steps.")
       while remaining_steps >= min_steps:
           add_steps = min(remaining_steps, min_stop_steps)
           steps_decelerate.append(add_steps)
#           print("Adding", add_steps, "steps at end of waveform")
           remaining_steps -= add_steps
       if remaining_steps > 0:
#           print("Finishing with", remaining_steps, "steps at end of waveform")
           steps_decelerate.append(remaining_steps)

    steps_accelerate.extend(steps_decelerate)

    # Check the step list achieves the target
    (steptotal, maxchange) = step_list_count(steps_accelerate)
    if ( steptotal != nsteps ):
        strg = "step_list_limacc: Failed to achieve target: %d steps planned, %s steps actual." % \
               (nsteps, steptotal)
        strg += "\n\tPlease raise a bug ticket."
        strg += "\n\t" + str(steps_accelerate)
        raise ValueError(strg)

    return steps_accelerate


def step_list_count(slist):
    # Count the number of steps moved by a particular step list
    # and return the count plus the maximum change between elements.
    nsteps = 0
    maxchange = 0
    laststep = 0
    for step in slist:
        nsteps += step
        if abs(step-laststep) > maxchange:
            maxchange = abs(step-laststep)
        laststep = step
    return (nsteps, maxchange)

def step_list_pad(slist, target_len):
    # Lengthen a step list by padding the end section with zeros.
    slist = list(slist)
    if len(slist) >= target_len:
        return slist
    dlen = target_len - len(slist)

    slist.reverse()
    slist.extend([ 0 ] * dlen)
    slist.reverse()
    return slist


def gen_slist(adegree,                     # Alpha angle to move
              bdegree,                     # Beta angle to move
              asteps_per_deg=None,         # Steps to degree conversion factor
              bsteps_per_deg=None,         # Steps to degree conversion factor
              mode=None,                   # *** NO LONGER NEEDED: Waveform mode
              units="degree",              # *** OVER-COMPLICATED: Units of adegree, bdegree
              max_change_alpha=None,
              max_acceleration_alpha=None, # Maximum acceleration alpha
              max_deceleration_alpha=None, # *** NO LONGER NEEDED: Maximum deceleration alpha
              min_steps_alpha=None,        # Minimum speed alpha
              min_stop_steps_alpha=None,   # Minimum stop speed alpha
              max_steps_alpha=None,        # Maximum speed alpha
              max_change_beta=None,        # *** NO LONGER NEEDED: obsolete parameter
              max_acceleration_beta=None,  # Maximum acceleration beta
              max_deceleration_beta=None,  # *** NO LONGER NEEDED: Maximum deceleration beta
              min_steps_beta=None,         # Minimum speed beta
              min_stop_steps_beta=None,    # Minimum stop speed beta
              max_steps_beta=None,         # Maximum speed alpha
              max_change=1.4,              # *** NO LONGER NEEDED: obsolete parameter
              max_acceleration=MOTOR_MAX_ACCELERATION,  # *** OVER-COMPLICATED. Default.
              max_deceleration=MOTOR_MAX_DECELERATION,  # *** OVER-COMPLICATED. Default
              min_steps=STEPS_LOWER_LIMIT, # *** OVER-COMPLICATED. Default.
              min_stop_steps=None,         # *** OVER-COMPLICATED. Default.
              max_steps=STEPS_UPPER_LIMIT, # *** OVER-COMPLICATED. Default.
              insert_rest_accelerate=None):# *** NO LONGER NEEDED: obsolete parameter
    """
       Generate alpha and beta angles for one FPU.
       See the gen_wf() function for a description of the function parameters.
       
    """
    # Sort out default values for parameters not included.
    # TODO: *** REMOVE THIS OVER-COMPLICATED CODE.
    # The default values should be defined in the function specification,
    # if at all.
    if min_steps_alpha is None:
        min_steps_alpha = min_steps
    if min_stop_steps_alpha is None:
        min_stop_steps_alpha = min_stop_steps
    if max_steps_alpha is None:
        max_steps_alpha = max_steps
    if max_change_alpha is None:
        max_change_alpha = max_change
    if max_acceleration_alpha is None:
        max_acceleration_alpha = max_acceleration
    if max_deceleration_alpha is None:
        max_deceleration_alpha = max_deceleration

    if min_steps_beta is None:
        min_steps_beta = min_steps
    if min_stop_steps_beta is None:
        min_stop_steps_beta = min_stop_steps
    if max_steps_beta is None:
        max_steps_beta = max_steps
    if max_change_beta is None:
        max_change_beta = max_change
    if max_acceleration_beta is None:
        max_acceleration_beta = max_deceleration
    if max_deceleration_beta is None:
        max_deceleration_beta = max_deceleration


    # TODO: *** REMOVE FROM C++ VERSION.
    # assert we don't deal with NaNs
    assert( (adegree == adegree) and (bdegree == bdegree))
    # (if the above code confuses you, read https://en.wikipedia.org/wiki/NaN
    # and https://docs.oracle.com/cd/E19957-01/806-3568/ncg_goldberg.html )
    
    # TODO: *** REMOVE. mode not needed.
    if not (mode in ['fast', 'slow', 'slowpar', 'limacc']):
            raise ValueError("mode needs to be one of 'slow', 'slowpar', 'fast', 'limacc'")

    # TODO: *** REMOVE. This function would be more modular if it always
    # operated in steps and units were managed by the caller.
    if units == "degree":
        asteps = int(round(adegree * asteps_per_deg))
        bsteps = int(round(bdegree * bsteps_per_deg))
    elif units == "steps":
        asteps = int(round(adegree))
        bsteps = int(round(bdegree))
    else:
        raise ValueError("The unit needs to be 'steps' or 'degree'")

    # Return +/- 1.0 depending on the sign of asteps or bsteps
    asign = int(math.copysign(1.0, asteps))
    bsign = int(math.copysign(1.0, bsteps))
    asteps *= asign # Result always positive. TODO: Would be easier to use abs()
    bsteps *= bsign # Result always positive. TODO: Would be easier to use abs()

    # TODO: *** REMOVE THIS TEST - MODE IS ALWAYS LINACC
    if mode in ['slow', 'slowpar']:
        # TODO: ***  REMOVE
        alist = step_list_slow(asteps, min_steps=min_steps_alpha)
        blist = step_list_slow(bsteps, min_steps=min_steps_beta)

        max_len = max(len(alist), len(blist))
        alist = step_list_pad(alist, max_len)
        blist = step_list_pad(blist, max_len)
        slist = [ (astep * asign, bstep * bsign)
                  for astep, bstep in zip(alist, blist) ]

    elif mode == 'fast':
        # TODO: ***  REMOVE
        alist = step_list_fast(asteps,
                               max_change=max_change_alpha,
                               min_steps=min_steps_alpha,
                               max_steps=max_steps_alpha)
        blist = step_list_fast(bsteps,
                               max_change=max_change_beta,
                               min_steps=min_steps_beta,
                               max_steps=max_steps_beta)

        max_len = max(len(alist), len(blist))
        alist = step_list_pad(alist, max_len)
        blist = step_list_pad(blist, max_len)

        slist = [ (astep * asign, bstep * bsign)
                  for astep, bstep in zip(alist, blist) ]
    else:
        # TODO: KEEP THIS SECTION ONLY
        alist = step_list_limacc(asteps,
                                 max_acceleration=max_acceleration_alpha,
                                 max_deceleration=max_deceleration_alpha,
                                 min_steps=min_steps_alpha,
                                 min_stop_steps=min_stop_steps_alpha,
                                 max_steps=max_steps_alpha,
                                 insert_rest_accelerate=insert_rest_accelerate)

        blist = step_list_limacc(bsteps,
                                 max_acceleration=max_acceleration_beta,
                                 max_deceleration=max_deceleration_beta,
                                 min_steps=min_steps_beta,
                                 min_stop_steps=min_stop_steps_beta,
                                 max_steps=max_steps_beta,
                                 insert_rest_accelerate=insert_rest_accelerate)

        # Pad the waveforms to the same length.
        max_len = max(len(alist), len(blist))
        alist = step_list_pad(alist, max_len)
        blist = step_list_pad(blist, max_len)

        slist = [ (astep * asign, bstep * bsign)
                  for astep, bstep in zip(alist, blist) ]


    if len(slist) == 0:
        slist = [ (0, 0) ]
    assert(len(slist) <= MAXNUM_WAVEFORM_SEGMENTS), "waveform exceeds maximum length (%d)" % MAXNUM_WAVEFORM_SEGMENTS

    return slist


def gen_wf(aangle,                                  # Alpha angle in degrees.
           bangle,                                  # Beta angle in degrees.
           asteps_per_deg=StepsPerDegreeAlpha,      # Steps to degree conversion
           bsteps_per_deg=StepsPerDegreeBeta,       # Steps to degree conversion
           units='degree',                          # TODO: Over-complication?
           mode='limacc',                           # TODO: *** REMOVE. NOT NEEDED.
           max_change=1.4,                          # TODO: *** REMOVE. NOT NEEDED.
           max_acceleration=MOTOR_MAX_ACCELERATION, # Maximum acceleration in steps/s/element
           max_deceleration=MOTOR_MAX_DECELERATION, # TODO: NOT NEEDED. Acceleration = deceleration
           min_steps=STEPS_LOWER_LIMIT,             # Min speed in steps/s
           max_steps=STEPS_UPPER_LIMIT,             # Max speed in steps/s
           **kwargs):
    """
    
    Generate a waveform which moves the alpha arm by an angle of
    adegree and the beta arm by bdegree. asteps_per_deg and bsteps_er_deg
    are approximate calibration factors. The mode parameter can be:

    - 'limacc' to generate a movement with constant acceleration/deceleration;
    - 'fast' to generate a movement with an exponential growth in speed; or
    - 'slow' or'slowpar' to generate a slow movement where alpha and beta
       are moved in parallel. (The former 'slow' mode is obsolete,
       it does not match the protocol and capabilities of the current
       firmware).

    If adegree or bdegree are arrays, extend then if possible to a
    common shape, and return a list of waveforms for a number of
    FPUs corresponding to the array.

    No range checking of movements is done.
    
    returns a dictionary where slist[N] contains [alphasteps, betasteps] for
    FPU index N. alphasteps and betasteps are lists of alpha and beta step
    counts.
    
    """
#    print("gen_wf: min_steps=%d, max_steps=%d, min_acceleration=%d, max_acceleration=%d" % \
#          (min_steps, max_steps, max_acceleration, max_deceleration) )
    # TODO: ALL MODES ARE OBSOLETE EXCEPT LINACC
    if mode == 'slow':
        warnings.warn("'slow' mode is obsolete, it does not match the waveform protocol, mapped to 'slowpar'.")

    # Ensure the alpha and beta angles are arrays of the correct size and shape
    aangle = asarray(aangle)
    bangle = asarray(bangle)

    if aangle.ndim == 0:
        aangle.shape = 1

    bangle = ones_like(aangle) * bangle
    aangle = ones_like(bangle) * aangle

    assert(aangle.ndim <= 1)

    # Generate a waveform for each FPU, i.
    slists = dict( (i, gen_slist(aangle[i], bangle[i], asteps_per_deg, bsteps_per_deg,
                                 mode=mode, units=units, max_change=max_change,
                                 min_steps=min_steps, max_steps=max_steps, **kwargs))
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


def read_wf(filename, **kwargs):
    print("""To read a waveform please use

w = wflib.load_waveform( filename )""")

def load_waveform(filename, **kwargs):
    print("""To read a waveform please use

w = wflib.load_waveform( filename )""")


def path_to_steps(p, steps_per_degree, origin=0.0):
    sum_steps = round((array(p,dtype=float) * RADIAN_TO_DEGREE - origin)
                      * steps_per_degree).astype(int)
    return sum_steps[1:] - sum_steps[:-1], sum_steps


if __name__ == '__main__':
    print("""Run this script with

   python -i fpu_commands.py

and use

   w = gen_wf( alpha_angles, beta_angles )
   s = step_list_limacc( nsteps )
   s = path_to_steps( path, steps_per_degree )

to test the waveform generation functions.
   """)

