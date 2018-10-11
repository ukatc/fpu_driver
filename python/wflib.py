from __future__ import print_function

from ast import literal_eval
from numpy import array, pi, round

from mocpath.path.path_generator import read_path_file

from fpu_constants import (ALPHA_DATUM_OFFSET, BETA_DATUM_OFFSET,
                           StepsPerDegreeAlpha, StepsPerDegreeBeta,
                           RADIAN_TO_DEGREE)

from fpu_commands import path_to_steps

def is_comment(s):
    """returns True if string s is a comment line
    """
    return s.strip()[0] == '#'


    

def load_paths(filename="targets_7fp_case_5_1_PATHS.txt",
                  canmap_fname="canmap.cfg", reverse=False):

    """Loads an outout file from the path generator, and returns
    a structure which can passed into the FPU driver's
    configPath() method.
    Caveat: Different from the path file, the angle units
    in the returned results are in radian."""
    
    paths = read_path_file(filename)

    idmap = literal_eval("".join(filter(lambda x: not is_comment(x),
                                        open(canmap_fname).readlines())))

    return { idmap[str(cellid)] : (alpha_path, beta_path)
             for cellid, alpha_path, beta_path  in paths }


def load_waveform(filename="targets_7fp_case_5_1_PATHS.txt",
                  canmap_fname="canmap.cfg", reverse=False):

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


def print_waveform(wf):
    for id, w in wf.items():
        print("-" * 30)
        print("fpu_id", id)
        for s in w:
            print("%5i,\t%5i" % (s[0],s[1]))
            
