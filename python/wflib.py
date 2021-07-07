from __future__ import print_function

from ast import literal_eval
from numpy import array, pi, round

# This module depends on the mocpath library, obtained from the
# MOONS software SVN repository.
from mocpath.path.path_generator import read_path_file

from fpu_constants import (ALPHA_DATUM_OFFSET, BETA_DATUM_OFFSET,
                           StepsPerDegreeAlpha, StepsPerDegreeBeta,
                           RADIAN_TO_DEGREE)

from fpu_commands import path_to_steps

def is_comment(s):
    """

    Returns True if string s is a comment line, i.e. a line
    beginning with a "#".

    """
    return s.strip()[0] == '#'


def load_paths(filename, canmap_fname="canmap.cfg", reverse=False):
    """

    Reads a PATHS file created by the path generator and
    returns a structure which can passed into the FPU driver's
    configPath() method.

    A configuration file (canmap_fname) provides the mapping
    between fibre positioner cell ID and FPU device driver ID.
    The canmap file is of the form:
    {  "<cell-id>" :  <fpu-id>,
       "<cell-id>" :  <fpu-id>,
       etc...
    }
    
    Caveat: Unlike those in the path file, the angle units
    in the returned results are in radians.

    """
    paths = read_path_file(filename)

    # Extract a dictionary from the canmap file by removing the
    # the comments and parsing it as a Python statement.
    # In the resulting dictionary, idmap["cell-id"] = fpu-id.
    idmap = literal_eval("".join(filter(lambda x: not is_comment(x),
                                        open(canmap_fname).readlines())))

    if len(idmap) < len(paths):
       strg = "ID mapping file %s is not long enough to define all the mappings needed." % canmap_fname
       strg += "\n\tThere are %d mappings in the configuration but %d paths." % (len(idmap), len(paths))
       raise ValueError(strg)

# The code in the return statement is a compressed version of the following.
#    path_dict = {}
#    for cellid, alpha_path, beta_path in paths:
#        fpu_id = idmap[str(cellid)]
#        path_dict[fpu_id] = (alpha_path, beta_path)
#    return path_dict

    try:
        return { idmap[str(cellid)] : (alpha_path, beta_path)
                 for cellid, alpha_path, beta_path in paths }
    except KeyError as e:
        strg = "Canmap file \'%s\' does not include all FPUs. No mapping for cell-id %s" % \
            (canmap_fname, str(e))
        raise KeyError(strg)


def load_waveform(filename, canmap_fname="canmap.cfg", reverse=False):
    """

    Reads a PATHS file created by the path generator and
    returns a structure which can passed into the FPU driver's
    configMotion() method.
    The returned results are in motor steps.

    """
    paths = load_paths(filename, canmap_fname)

    waveform = {}
    for fpu_id, (alpha_path, beta_path)  in paths.items():
        alpha_steps, _ = path_to_steps(alpha_path, StepsPerDegreeAlpha,
                                    origin=ALPHA_DATUM_OFFSET)
        beta_steps, _ = path_to_steps(beta_path, StepsPerDegreeBeta,
                                   origin=BETA_DATUM_OFFSET)

        tseries = zip(alpha_steps, beta_steps)
        if reverse:
            tseries.reverse()
        waveform[fpu_id] = tseries
        
    
    return waveform

def filter_array( values, scale=5):
    """

    Filter a single array of motor step values.
    Patterns of the form 0 +N 0 -N 0 will be replaced by 0 0 0 0 0.

    """
    oldsum = sum(values)

    # Step through the array looking for segements of the give scale length
    # which begin and end in a zero.
    first_element = scale
    last_element = len(values) - scale
    element = first_element

    while ( element < last_element ):
        #print("Testing element %d" % element )
        endsegment = element + scale - 1
        if values[element] == 0 and values[endsegment] == 0:
            # Both ends are zero. Look for the 0 +N 0 -N 0 pattern
            # and filter it out.
            #print("Elements %d and %d are both zero." % (element, endsegment))
            total = 0.0
            for ii in range(element+1, endsegment):
                total += values[ii]

            if total == 0:
                #print("Central section also zero. Filtering.")
                for ii in range(element+1, endsegment):
                    #print("[%d]: Replacing %d with 0" % (ii, values[ii]))
                    values[ii] = 0
            element += scale  # Skip to the next segment
        else:
            element += 1      # No pattern found. Try the next element.

    newsum = sum(values)
    # The total length of the path in steps must not change
    assert newsum == oldsum, "***ERROR: Waveform length changed during filtering."
    return values


def filter_waveform_pos(wfpos, scale=5):
    """

    Filter the waveform for a single positioner.
    See filter_waveform function for details.

    """
    # Deconstruct the waveform into a separate alpha and beta arrays.
    alpha = []
    beta = []
    for s in wfpos:
        alpha.append( s[0])
        beta.append( s[1] )

    # Filter each array in turn.
    newalpha = filter_array( alpha, scale=scale)
    newbeta = filter_array( beta, scale=scale)

    # Now reconstruct the waveform data structure and return it.
    newwfpos = []
    for a, b in zip(newalpha, newbeta):
        newwfpos.append( (a, b) )
    return newwfpos


def filter_waveform(wf, scale=5):
    """

    Filter a waveform by looking for repeated patterns that make no
    progress and can be eliminated without violating speed and
    acceleration requirements. For example (for scale=5):

    [...0, +N, 0, -N, 0...] --> [...0, 0, 0, 0, 0...]

    The best filtering can be achieved by filtering first at a scale
    length of 13 and filtering again at a scale length of 5.

    """
    newwf = {}
    for idkw in list(wf.keys()):
        # Filter each waveform in turn and add the new waveform
        # to the new dictionary.
        newwf[idkw] = filter_waveform_pos( wf[idkw], scale=scale )
    return newwf


def print_waveform(wf, columns=True):
    """

    Prints the contents of a waveform in a human-readable form.
    The columns flag determines whether steps are printed in
    rows or columns.

    """
    for id, w in wf.items():
        print("-" * 30)
        print("fpu_id", id)
        if columns:
            for s in w:
                print("%5i,\t%5i" % (s[0],s[1]))
        else:
            strg1 = ""
            strg2 = ""
            for s in w:
                strg1 += "%5i, " % s[0]
                strg2 += "%5i, " % s[1]
            print( "alpha: " + strg1 )
            print( "beta: " + strg2 )


if __name__ == '__main__':
    print("""Run this script with

   python -i wflib.py

and use commands such as

   p = load_paths( filename, canmap )
   w = load_waveform( filename, canmap )
   f = filter_waveform( w )
   print_waveform( w )

to test the reading of path analysis PATH files
and processing of waveforms.
   """)
