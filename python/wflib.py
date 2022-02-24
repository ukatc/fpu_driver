from __future__ import print_function

from ast import literal_eval
from numpy import array, pi, round

# This module depends on the mocpath library, obtained from the
# MOONS software SVN repository.
import mocpath.util as util
import mocpath.path.pa_parameters as params
import mocpath.path.path_generator as pg
from mocpath.path.pa_dnf import analyse_dnf

from fpu_constants import (ALPHA_DATUM_OFFSET, BETA_DATUM_OFFSET,
                           StepsPerDegreeAlpha, StepsPerDegreeBeta,
                           RADIAN_TO_DEGREE, DEGREE_TO_RADIAN)

from fpu_commands import path_to_steps

from FpuGridDriver import REQD_CLOCKWISE, REQD_ANTI_CLOCKWISE, \
                          DIRST_CLOCKWISE, DIRST_ANTI_CLOCKWISE


def is_comment(s):
    """

    Returns True if string s is a comment line, i.e. a line
    beginning with a "#".

    """
    return s.strip()[0] == '#'


def direction_to_value( direction_name ):
    # Convert the name of a direction into a grid driver value
    if direction_name.strip().upper() == "ACW":
        direction_value = REQD_ANTI_CLOCKWISE
    elif direction_name.strip().upper() == "CW":
        direction_value = REQD_CLOCKWISE
    else:
        direction_value = None
    return direction_value


def plot_geometry( config_file, canmap_fname, arm_angles ):
    """
    
    Display the geometry of the grid layout defined in the configuration
    file and the arm angles extracted from the grid state
    
    """
    # Extract a dictionary from the canmap file by removing the
    # the comments and parsing it as a Python statement.
    # In the resulting dictionary, idmap["cell-id"] = fpu-id.
    idmap = literal_eval("".join(filter(lambda x: not is_comment(x),
                                        open(canmap_fname).readlines())))
    # Reverse the dictionary so that reversemap["fpu-id"] = cell-id
    reversemap = {}
    for fpuid in list(idmap.keys()):
        cellid = idmap[fpuid]
        reversemap[str(cellid)] = int(fpuid)

    # Use the canmap dictionary to process the IDs within arm_angles
    new_arm_angles = []
    for arms in arm_angles:
        fpuid = arms[0]
        cellid = reversemap[str(fpuid)]
        #print(fpuid, "-->", cellid)
        new_arm_angles.append( (cellid, arms[1], arms[2]))

    #---------------------------------------------------------------------
    # Create a fibre positioner grid based on the configuration information.
    #---------------------------------------------------------------------
    positioner_grid = pg.create_positioner_grid( config_file )
    pg.define_arm_angles( new_arm_angles, positioner_grid )
    
    # Plot the current situation.
    strg = "Fibre positioner grid: %s" % config_file
    try:
        positioner_grid.plot(description=strg, targetlist=[])
    except:
        # Ignore exceptions thrown by the plotting.
        pass
    
    return


def generate_safe_paths( config_file, canmap_fname, arm_angles, target="SAFE",
                         brute_force_elements=None, verbose=False, plot=False ):
    """
    
    Generate a path that will move the fibre positioners to safe
    locations.
    
    """
    if brute_force_elements is None:
        brute_force_elements = params.BRUTE_FORCE_ELEMENTS_DEFAULT

    # Extract a dictionary from the canmap file by removing the
    # the comments and parsing it as a Python statement.
    # In the resulting dictionary, idmap["cell-id"] = fpu-id.
    idmap = literal_eval("".join(filter(lambda x: not is_comment(x),
                                        open(canmap_fname).readlines())))
    # Reverse the dictionary so that reversemap["fpu-id"] = cell-id
    reversemap = {}
    for fpuid in list(idmap.keys()):
        cellid = idmap[fpuid]
        reversemap[str(cellid)] = int(fpuid)

    # Use the canmap dictionary to process the IDs within arm_angles
    new_arm_angles = []
    for arms in arm_angles:
        fpuid = arms[0]
        cellid = reversemap[str(fpuid)]
        #print(fpuid, "-->", cellid)
        new_arm_angles.append( (cellid, arms[1], arms[2]))

    #---------------------------------------------------------------------
    # Create a fibre positioner grid based on the configuration information.
    #---------------------------------------------------------------------
    positioner_grid = pg.create_positioner_grid( config_file,
                                                 compress_limits=True )

    # Start plotting the path from this location
    pg.define_arm_angles( new_arm_angles, positioner_grid )
    
    # Generate a set of default targets
    if target == "LOW":
        default_targets = pg.generate_low_targets( positioner_grid )
    else:
        default_targets = pg.generate_default_targets( positioner_grid )

#     # Plot the initial situation.
#     if plot:
#         strg = "Initial situation: %s" % config_file
#         try:
#             positioner_grid.plot(description=strg, targetlist=[])
#         except:
#             # Ignore exceptions thrown by the plotting.
#             pass

    if target.upper() == 'SAFE':
        # Alpha arms avoid other positioners and beta arms tuck in.
        goto = [params.NO_TARGET, params.GOTO_ZERO]
        repulsion_factor = 4.0

    elif target.upper() == 'SPACE':
        # Both arms avoid other positioners.
        goto = [params.NO_TARGET, params.NO_TARGET]
        repulsion_factor = 10.0
        
    else:
        # Move to DEFAULT or LOW. Alpha and beta targets both active.
        goto = [params.GOTO_TARGET, params.GOTO_TARGET]
        repulsion_factor = 4.0

    # Use the DNF or BRUTEFORCE algorithm to plot a path to safety
    (actual_target_list, positioner_paths) = \
            analyse_dnf(default_targets, positioner_grid, safe_start=False,
                        generate_paths=True, goto=goto,
                        brute_force_elements=brute_force_elements,
                        repulsion_factor=repulsion_factor )

    if plot:
        # If required, plot the final initial situation.
        strg = "Fibre positioners with safe paths: %s" % config_file
        try:
            positioner_grid.plot(description=strg, targetlist=[], withpath=True)
        except:
            # Ignore exceptions thrown by the plotting.
            pass

    return positioner_paths


def recovery_directions( config_file, canmap_fname, arm_angles, max_steps=6,
                         verbose=False, plot=False ):
    """
    
    Determine the recovery directions using the grid layout defined in the
    configuration file and the arm angles extracted from the grid state
    
    """
    # Extract a dictionary from the canmap file by removing the
    # the comments and parsing it as a Python statement.
    # In the resulting dictionary, idmap["cell-id"] = fpu-id.
    idmap = literal_eval("".join(filter(lambda x: not is_comment(x),
                                        open(canmap_fname).readlines())))
    # Reverse the dictionary so that reversemap["fpu-id"] = cell-id
    reversemap = {}
    for fpuid in list(idmap.keys()):
        cellid = idmap[fpuid]
        reversemap[str(cellid)] = int(fpuid)

    # Use the canmap dictionary to process the IDs within arm_angles
    new_arm_angles = []
    for arms in arm_angles:
        fpuid = arms[0]
        cellid = reversemap[str(fpuid)]
        #print(fpuid, "-->", cellid)
        new_arm_angles.append( (cellid, arms[1], arms[2]))

    #---------------------------------------------------------------------
    # Create a fibre positioner grid based on the configuration information.
    #---------------------------------------------------------------------
    positioner_grid = pg.create_positioner_grid( config_file )
    pg.define_arm_angles( new_arm_angles, positioner_grid )
    
    # Plot the current situation if requested
    if plot:
        strg = "Fibre positioner grid: %s" % config_file
        try:
            positioner_grid.plot(description=strg, targetlist=[])
        except:
            # Ignore exceptions thrown by the plotting.
            pass
    
    directions = pg.recovery_directions( positioner_grid, new_arm_angles,
                                         max_steps=max_steps, verbose=verbose )
    #print("directions:", directions)
    
    # Convert the cell IDs contained in the mocpath instructions back
    # into CAN IDs and convert the direction names into integer
    # instructions.
    new_directions = {}
    for cellid in list(directions.keys()):
        fpuid = idmap[str(cellid)]
        new_dir = directions[cellid]
        alpha_dir = direction_to_value(new_dir[0])
        beta_dir = direction_to_value(new_dir[2])
        new_directions[fpuid] =  (alpha_dir, new_dir[1], beta_dir, new_dir[3], new_dir[4] )
    #print("new_directions:", new_directions)
    return new_directions 


def convert_paths(paths, canmap_fname="canmap.cfg", reverse=False):

    """

    Converts a mocpath data structure containing fibre positioner paths
    into a structure which can passed into the FPU driver's
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
    paths = pg.read_path_file(filename)
    return convert_paths( paths, canmap_fname=canmap_fname, reverse=reverse)

#     # Extract a dictionary from the canmap file by removing the
#     # the comments and parsing it as a Python statement.
#     # In the resulting dictionary, idmap["cell-id"] = fpu-id.
#     idmap = literal_eval("".join(filter(lambda x: not is_comment(x),
#                                         open(canmap_fname).readlines())))
# 
#     if len(idmap) < len(paths):
#        strg = "ID mapping file %s is not long enough to define all the mappings needed." % canmap_fname
#        strg += "\n\tThere are %d mappings in the configuration but %d paths." % (len(idmap), len(paths))
#        raise ValueError(strg)
# 
# # The code in the return statement is a compressed version of the following.
# #    path_dict = {}
# #    for cellid, alpha_path, beta_path in paths:
# #        fpu_id = idmap[str(cellid)]
# #        path_dict[fpu_id] = (alpha_path, beta_path)
# #    return path_dict
# 
#     try:
#         return { idmap[str(cellid)] : (alpha_path, beta_path)
#                  for cellid, alpha_path, beta_path in paths }
#     except KeyError as e:
#         strg = "Canmap file \'%s\' does not include all FPUs. No mapping for cell-id %s" % \
#             (canmap_fname, str(e))
#         raise KeyError(strg)


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
    pg.save_targets_from_angles( status_file, config_file, arm_angles_rad )
        
if __name__ == '__main__':
    print("""Run this script with

   python -i wflib.py

and use commands such as

   dirs = recovery_directions( config_file, canmap_fname, arm_angles )
   p = load_paths( filename )
   w = load_waveform( filename )
   print_waveform( w )

to test the geometry functions and reading of path analysis PATH files.
   """)
