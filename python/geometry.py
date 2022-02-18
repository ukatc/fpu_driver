from __future__ import print_function

from ast import literal_eval
#from numpy import array, pi, round

# This module depends on the mocpath library, obtained from the
# MOONS software SVN repository.
import mocpath.util as util
import mocpath.path.path_generator as pg

from FpuGridDriver import REQD_CLOCKWISE, REQD_ANTI_CLOCKWISE, \
                          DIRST_CLOCKWISE, DIRST_ANTI_CLOCKWISE

#from fpu_constants import (ALPHA_DATUM_OFFSET, BETA_DATUM_OFFSET,
#                           StepsPerDegreeAlpha, StepsPerDegreeBeta,
#                           RADIAN_TO_DEGREE, DEGREE_TO_RADIAN)

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


if __name__ == '__main__':
    print("""Run this script with

   python -i geometry.py

and use commands such as

   dirs = recovery_directions( config_file, canmap_fname, arm_angles )

to test the geometry functions.
   """)
