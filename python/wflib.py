from __future__ import print_function

from ast import literal_eval
from numpy import array, pi, round

# This module depends on the mocpath library, obtained from the
# MOONS software SVN repository.
from mocpath.path.path_generator import read_path_file, save_targets_from_angles

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

def save_angles_to_file( arm_angles, status_file, config_file,
                         canmap_fname="canmap.cfg" ): 

    # Extract a dictionary from the canmap file by removing the
    # the comments and parsing it as a Python statement.
    # In the resulting dictionary, idmap["cell-id"] = fpu-id.
    idmap = literal_eval("".join(filter(lambda x: not is_comment(x),
                                        open(canmap_fname).readlines())))

    if len(idmap) < len(arm_angles):
       strg = "ID mapping file %s is not long enough to define all the mappings needed." % canmap_fname
       strg += "\n\tThere are %d mappings in the configuration but %d arm angles." % (len(idmap), len(arm_angles))
       raise ValueError(strg)

    def fpuid_to_cellid( fid ):
        for index in list(idmap.keys()):
            if fid == idmap[index]:
                return int(index)

    # Translate the arm angles and convert to radians.
    arm_angles_rad = []
    for fpuid, alpha_deg, beta_deg in arm_angles:
        cell_id = fpuid_to_cellid( fpuid )
        arm_angles_rad.append( [cell_id,
                                alpha_deg/RADIAN_TO_DEGREE,
                                beta_deg/RADIAN_TO_DEGREE])

    # Save those angles as target locations for the given configuration file
    save_targets_from_angles( status_file, config_file, arm_angles_rad )
        
if __name__ == '__main__':
    print("""Run this script with

   python -i wflib.py

and use commands such as

   p = load_paths( filename )
   w = load_waveform( filename )
   print_waveform( w )

to test the reading of path analysis PATH files.
   """)
