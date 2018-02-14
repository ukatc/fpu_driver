#!/usr/bin/python
from __future__ import division, print_function

import math

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
    return [ (gs.FPU[i].alpha_steps, gs.FPU[i].beta_steps)
             for i in range(num_fpus)]

def fpu_steps(gs):
    return list_positions(gs)[0]


def list_angles(gs, asteps_per_deg=125, bsteps_per_deg=80, num_fpus=None):
    """Show approximate angular positions for each FPU in the grid. 
       The optional second and third argument are the scaling factors,
       and the fourth argument is the number of FPUs shown."""
    if num_fpus == None:
        num_fpus = len(gs.FPU)
    # the following line uses Python3 float division
    return [ (gs.FPU[i].alpha_steps / asteps_per_deg,
              gs.FPU[i].beta_steps / bsteps_per_deg) for i in range(num_fpus)]


def fpu_ang(gs, asteps_per_deg=125, bsteps_per_deg=80,):
    return list_angles(gs, asteps_per_deg=asteps_per_deg,
                       bsteps_per_deg=bsteps_per_deg)[0]

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



def gen_wf(adegree, bdegree, asteps_per_deg=125, bsteps_per_deg=80):
    asteps = adegree * asteps_per_deg
    bsteps = bdegree * bsteps_per_deg
    max_steps = max(asteps, bsteps)
    nsegments = int(math.ceil(max_steps / 100.0))
    adelta = asteps / nsegments
    bdelta = bsteps / nsegments
    slist = [ (adelta, bdelta) for k in range(nsegments) ]
    return { 0 : slist }
