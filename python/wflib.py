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
    """returns True if string s is a comment line
    """
    return s.strip()[0] == '#'


    

def load_paths(filename, canmap_fname="canmap.cfg", reverse=False):

    """Reads a PATHS file created by the path generator and
    returns a structure which can passed into the FPU driver's
    configPath() method.

    A configuration file (canmap_fname) provides the mapping
    between fibre positioner ID and FPU device driver ID.

    Caveat: Unlike those in the path file, the angle units
    in the returned results are in radians."""
    
    paths = read_path_file(filename)

    idmap = literal_eval("".join(filter(lambda x: not is_comment(x),
                                        open(canmap_fname).readlines())))

    if len(idmap) < len(paths):
       strg = "ID mapping file %s is not long enough to define all the mappings needed." % canmap_fname
       strg += "\n\tThere are %d mappings in the configuration but %d paths." % (len(idmap), len(paths))
       raise ValueError(strg)

    return { idmap[str(cellid)] : (alpha_path, beta_path)
             for cellid, alpha_path, beta_path  in paths }


def load_waveform(filename, canmap_fname="canmap.cfg", reverse=False):

    """Reads a PATHS file created by the path generator and
    returns a structure which can passed into the FPU driver's
    configMotion() method.
    The returned results are in motor steps."""

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


def print_waveform(wf, columns=True):
    """Prints the contents of a waveform in a human-readable form.
    The columns flag determines whether steps are printed in
    rows or columns"""
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

   p = load_paths( filename )
   w = load_waveform( filename )
   print_waveform( w )

to test the reading of path analysis PATH files.
   """)
