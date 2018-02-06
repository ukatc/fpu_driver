#!/usr/bin/python

""" Utility functions for using the fpu_driver module on the command line.
"""

""" getGridStateSummary takes a gridstate (gs) and returns
    a summary state defined by the lowest commin denominator of each 
    FPUs state."""

from fpu_driver import getGridStateSummary as gGSS


def list_positions(gs, num_fpus=None):
    """Show positions for each FPU in the grid. The optional second argument
       is the number of FPUs shown."""
    if num_fpus == None:
        num_fpus = len(gs.FPU)
    return [ (gs.FPU[i].alpha_steps, gs.FPU[i].beta_steps) for i in range(num_fpus)]

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

