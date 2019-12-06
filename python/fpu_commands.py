#!/usr/bin/python
from __future__ import division, print_function

import warnings

import math
import bisect
from numpy import array, asarray, ones_like, ceil, floor, round

""" Utility functions for using the fpu_driver module on the command line.
"""

""" getGridStateSummary takes a gridstate (gs) and returns
    a summary state defined by the lowest commin denominator of each
    FPUs state."""


from fpu_constants import *

from ethercanif import getGridStateSummary as gGSS




def list_positions(gs, num_fpus=None, show_zeroed=True):
    """Show positions for each FPU in the grid. The optional second argument
       is the number of FPUs shown."""
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
                num_fpus=None):
    """Show approximate angular positions for each FPU in the grid.
       The optional second and third argument are the scaling factors,
       and the fourth argument is the number of FPUs shown."""
    if num_fpus == None:
        num_fpus = len(gs.FPU)

    nan = float('NaN')
    tvalid = lambda zeroed : 1.0 if (zeroed or show_uninitialized) else nan
    # the following line uses Python3 float division
    return [ ((gs.FPU[i].alpha_steps / asteps_per_deg + alpha_datum_offset)* tvalid(gs.FPU[i].alpha_was_referenced),
              gs.FPU[i].beta_steps / bsteps_per_deg * tvalid(gs.FPU[i].beta_was_referenced)) for i in range(num_fpus)]


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


def step_list_limacc(nsteps, max_acceleration=MOTOR_MAX_ACCELERATION,
                     max_deceleration=MOTOR_MAX_DECELERATION,
                     min_steps=STEPS_LOWER_LIMIT,
                     min_stop_steps=STEPS_LOWER_LIMIT,
                     max_steps=STEPS_UPPER_LIMIT,
                     insert_rest_accelerate=True):

    rest_steps = nsteps
    steps_accelerate = []
    steps_decelerate = []

    if min_stop_steps is None:
        min_stop_steps = min_steps

    new_speed = [min_steps, min_stop_steps]
    max_change = [max_acceleration, max_deceleration]
    max_speed = [min_steps + max_acceleration, min_stop_steps + max_deceleration]
    steps = [[],[]]
    ACC = 0
    DEC = 1
    while True:
        if rest_steps < min(min_steps, min_stop_steps):
            break

        if len(steps[ACC]) == 0:
            W = ACC
            X = DEC
        elif len(steps[DEC]) == 0:
            W = DEC
            X = ACC
        elif new_speed[ACC] <= new_speed[DEC]:
            W = ACC
            X = DEC
        else:
            W = DEC
            X = ACC



        if  new_speed[W] > rest_steps:
            # we can't accelerate more
            break

        # compute new speed from acceleration limit
        if len(steps[W]) == 0:
            if W == ACC:
                tent_new_speed = min_steps
            else:
                tent_new_speed = min_stop_steps
        else:
            tent_new_speed = new_speed[W] + max_change[W]
        # check for max speed limit
        if tent_new_speed > max_steps:
            tent_new_speed = max_steps


        # if speed exhausts available distance, cap speed
        # to the smaller of rest distance, or counter-acting
        # change
        if (tent_new_speed + max_speed[X]  > rest_steps) :
            tent_new_speed = min(tent_new_speed, max_speed[X])

        tent_new_speed = min(tent_new_speed, rest_steps)

        # accept new speed step
        new_speed[W] = tent_new_speed
        steps[W].append(new_speed[W])
        max_speed[W] = tent_new_speed + max_change[W]
        rest_steps -= new_speed[W]


    if insert_rest_accelerate:
        W = ACC
    else:
        W = DEC


    if W == ACC:
        max_speed = min_steps
    else:
        max_speed = min_stop_steps

    if len(steps[W]) > 0:
        max_speed = steps[W][-1]


    while rest_steps >= min_steps:
        ins_steps = min(rest_steps, max_speed)
        bisect.insort(steps[W], ins_steps)
        rest_steps -= ins_steps

    steps_accelerate = steps[ACC]
    steps_decelerate = steps[DEC]
    steps_decelerate.reverse()


    if rest_steps > 0:
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
              mode=None, units="degree",
              max_change_alpha=None,
              max_acceleration_alpha=None,
              max_deceleration_alpha=None,
              min_steps_alpha=None,
              min_stop_steps_alpha=None,
              max_steps_alpha=None,
              max_change_beta=None,
              max_acceleration_beta=None,
              max_deceleration_beta=None,
              min_steps_beta=None,
              min_stop_steps_beta=None,
              max_steps_beta=None,
              max_change=1.2,
              max_acceleration=MOTOR_MAX_ACCELERATION,
              max_deceleration=MOTOR_MAX_DECELERATION,
              min_steps=STEPS_LOWER_LIMIT,
              min_stop_steps=None,
              max_steps=STEPS_UPPER_LIMIT):

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


    # assert we don't deal with NaNs
    assert( (adegree == adegree) and (bdegree == bdegree))
    # (if the above code confuses you, read https://en.wikipedia.org/wiki/NaN
    # and https://docs.oracle.com/cd/E19957-01/806-3568/ncg_goldberg.html )
    if not (mode in ['fast', 'slow', 'slowpar', 'limacc']):
            raise ValueError("mode needs to be one of 'slow', 'slowpar', 'fast', 'limacc'")

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
        alist = step_list_slow(asteps, min_steps=min_steps_alpha)
        blist = step_list_slow(bsteps, min_steps=min_steps_beta)

        max_len = max(len(alist), len(blist))
        alist = step_list_pad(alist, max_len)
        blist = step_list_pad(blist, max_len)
        slist = [ (astep * asign, bstep * bsign)
                  for astep, bstep in zip(alist, blist) ]

    elif mode == 'fast':
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
        alist = step_list_limacc(asteps,
                                 max_acceleration=max_acceleration_alpha,
                                 max_deceleration=max_deceleration_alpha,
                                 min_steps=min_steps_alpha,
                                 min_stop_steps=min_stop_steps_alpha,
                                 max_steps=max_steps_alpha)

        blist = step_list_limacc(bsteps,
                                 max_acceleration=max_acceleration_beta,
                                 max_deceleration=max_deceleration_beta,
                                 min_steps=min_steps_beta,
                                 min_stop_steps=min_stop_steps_beta,
                                 max_steps=max_steps_beta)

        max_len = max(len(alist), len(blist))
        alist = step_list_pad(alist, max_len)
        blist = step_list_pad(blist, max_len)

        slist = [ (astep * asign, bstep * bsign)
                  for astep, bstep in zip(alist, blist) ]


    if len(slist) == 0:
        slist = [ (0, 0) ]
    assert(len(slist) <= MAXNUM_WAVEFORM_SEGMENTS), "waveform exceeds maximum length (%d)" % MAXNUM_WAVEFORM_SEGMENTS

    return slist


def gen_wf(aangle, bangle, asteps_per_deg=StepsPerDegreeAlpha,
           bsteps_per_deg=StepsPerDegreeBeta,
           units='degree',
           mode='limacc',
           max_change=1.2,
           max_acceleration=MOTOR_MAX_ACCELERATION,
           max_deceleration=MOTOR_MAX_DECELERATION,
           min_steps=STEPS_LOWER_LIMIT,
           max_steps=STEPS_UPPER_LIMIT,
           **kwargs):
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
    
    returns a dictionary where slist[N] contains [alphasteps, betasteps] for
    FPU index N. alphasteps and betasteps are lists of alpha and beta step
    counts.
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

def read_path_file_text(path_file):
    """
    
    Read a file containing the motor paths needed to move the
    fibre positioners from their starting position to their finishing
    position.
    
    Motor angles are stored in the file in degrees.
    
    :Parameters:
        
    path_file: str
        The name of the path file to be read.
   
    :Returns:

    fpu_paths: list of (id, motor1_array, motor2_array)
        List of motor paths read from the file.
    
    """
    fpu_paths = []
    lastid = -1
    array1 = []
    array2 = []
    # Open the paths file within a Python context.
    with open(path_file, 'r') as file_in:
        # Read the ID and alpha and beta paths for each positioner from the file
        while True:
            line = file_in.readline()
            if line != '\n' and not line:
                # Empty line means end of file
                break
            if not line.strip():
                # Skip a blank line
                continue
            if "Positioner" in line:
                # New positioner
                if lastid > 0:
                    #print("Appending path for positioner", lastid)
                    fpu_paths.append( [lastid, array1, array2] )
                line = line.replace('\"', '') # Remove quotes
                words = line.split()
                lastid = int(words[1])
                array1 = []
                array2 = []
                #print("New positioner:", lastid)
            elif "[" in line:
                line = line.replace('[', '')
                line = line.replace(']', '') # Entire array might be on same line
                line = line.replace(',', ' ') # Older formats included commas
                words = line.split()
                if not array1:
                    # New array1
                    for wrd in words:
                        array1.append( float(wrd) )
                    #print("[ Starting new alpha array of length", len(array1))
                elif not array2:
                    # New array2
                    for wrd in words:
                        array2.append( float(wrd) )
                    #print("[ Starting new beta array of length", len(array2))
#             elif "]" in line: # ??
            else:
                line = line.replace(']', '')
                line = line.replace(',', ' ')
                words = line.split()
                # Add next line to array
                if not array2:
                    # Building up array1
                    for wrd in words:
                        array1.append( float(wrd) )
                    #print("Building up array1 to make length",  len(array1))
                else:
                    # Building up array2
                    for wrd in words:
                        array2.append( float(wrd) )
                    #print("Building up array2 to make length",  len(array2))
        # Append the last item.         
        #print("Appending path for positioner", lastid)
        fpu_paths.append( [lastid, array1, array2] )
    file_in.close()
    return fpu_paths

def read_path_file_paths(path_file):
    """
    
    Read a file containing the motor paths needed to move the
    fibre positioners from their starting position to their finishing
    position.
    
    Motor angles are stored in the file in degrees.

    This version reads motor paths from a new text file format which is
    more easily created and parsed in C++
    
    :Parameters:
        
    path_file: str
        The name of the path file to be read.
   
    :Returns:

    fpu_paths: list of (id, motor1_array, motor2_array)
        List of motor paths read from the file.
    
    """
    fpu_paths = []
    openid = -1
    closeid = -1
    array1 = []
    array2 = []
    # Open the paths file within a Python context.
    with open(path_file, 'r') as file_in:
        # Read the ID and alpha and beta paths for each positioner from the file
        while True:
            line = file_in.readline()
            if line != '\n' and not line:
                # Empty line means end of file
                break
            # Remove any comments after a '#' character.
            cn = line.find('#')
            if cn >= 0:
                line = line[:cn]
            if not line.strip():
                # Skip a blank line
                continue

            if "{HDR" in line:
                # Read the header keywords. (For now just skip them.)
                # TODO: Parse the header lines
                while "}HDR" not in line:
                    #print("Skipping header line:", line)
                    line = file_in.readline()
                    if line != '\n' and not line:
                        # Empty line means end of file.
                        # TODO: Error message?
                        break
                    if "}HDR" in line:
                        # End of header
                        break
                # Header completed. Move to the next line.
                continue

            if "{POS" in line:
                # New positioner
                words = line.split()
                openid = int(words[1])
                array1 = []
                array2 = []
                #print("New positioner:", openid)
            elif "alpha=[" in line:
                line = line.replace('alpha=[', '')
                line = line.replace(']', '') # Entire array might be on same line
                words = line.split()
                # New array1
                for wrd in words:
                    array1.append( float(wrd) )
                #print("New alpha array with starting length", len(array1))
            elif "beta=[" in line:
                line = line.replace('beta=[', '')
                line = line.replace(']', '') # Entire array might be on same line
                words = line.split()
                # New array2
                for wrd in words:
                    array2.append( float(wrd) )
                #print("New beta array with starting length", len(array2))
            elif "}POS" in line:
                # Closing a positioner section
                words = line.split()
                closeid = int(words[1])
                if openid > 0:
                    if closeid == openid:
                        #print("Appending path for positioner", openid)
                        fpu_paths.append( [openid, array1, array2] )
                    else:
                        logger.error("Parse error: Section opened with %d closes with %d." % \
                                 (openid, closeid))
                else:
                    logger.error("Parse error: Closing section for %d found without opening section." % \
                        closeid)
            else:
                # An an-between line containing additional array values.
                # Erase the ']' of the line contains the last part of an array.
                line = line.replace(']', '')
                words = line.split()
                # Add next line to array
                if not array2:
                    # Building up array1
                    for wrd in words:
                        array1.append( float(wrd) * params.DEG_TO_RAD )
                    #print("Building up array1 to make length",  len(array1))
                else:
                    # Building up array2
                    for wrd in words:
                        array2.append( float(wrd) * params.DEG_TO_RAD )
                    #print("Building up array2 to make length",  len(array2))

        # If necessary, add the last item.
        if openid > closeid:
            logger.warn("Final section of path file not closed")
            print("Appending path for positioner", lastid)
            fpu_paths.append( [openid, array1, array2] )

    file_in.close()
    return fpu_paths

def read_path_file_json(path_file):
    """
    
    Read a file containing the motor paths needed to move the
    fibre positioners from their starting position to their finishing
    position.
    
    Motor angles are stored in the file in degrees.

    NOTE: New version which reads the paths in JSON format.
    
    :Parameters:
        
    path_file: str
        The name of the path file to be read.
   
    :Returns:

    fpu_paths: list of (id, motor1_array, motor2_array)
        List of motor paths read from the file.
    
    """
    import json
    
    fpu_paths = []
    array1 = []
    array2 = []
    # Open the paths file within a Python context.
    with open(path_file, 'r') as json_file_in:
        pathdata = json.load( json_file_in )
        for path in pathdata['paths']:
            ident = path['ident']
            array1 = np.array( path['alpha'] )
            array2 = np.array( path['beta'] )
            fpu_paths.append( [ident, array1, array2] )
    json_file_in.close()
    return fpu_paths

def read_wf(filename, max_fpu, asteps_per_deg=StepsPerDegreeAlpha,
           bsteps_per_deg=StepsPerDegreeBeta, **kwargs):
    """
    
    Read the paths from the given file and convert then to waveforms. 
    max_fpu can be used to limit the number of waveforms extracted
    
    """
    # The filename must be a non-empty string
    assert(isinstance(filename, str)), "filename must be a string"
    assert(len(filename) > 0), "empty filename"

    # The file name extension indicates the type of the path file.
    if filename.lower().endswith('.json'):
        logger.debug("Reading JSON file: %s" % filename)
        fpu_paths = read_path_file_json(filename)
    elif filename.lower().endswith('.paths'):
        logger.debug("Reading PATHS file: %s" % filename)
        fpu_paths = read_path_file_paths(filename)
    else:
        logger.debug("Reading TEXT file: %s" % filename)
        fpu_paths = read_path_file_text(filename)

    # Convert the FPU paths into a steplist
    slists = {}
    for (fpu_ident, alphadegrees, betadegrees) in fpu_paths:
        slists[fpu_ident] = [alphadegrees * asteps_per_deg, betadegrees * bsteps_per_deg]
    return slists

def path_to_steps(p, steps_per_degree, origin=0.0):
    sum_steps = round((array(p,dtype=float) * RADIAN_TO_DEGREE - origin)
                      * steps_per_degree).astype(int)
    return sum_steps[1:] - sum_steps[:-1], sum_steps
