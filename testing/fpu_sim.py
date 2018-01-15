#!/usr/bin/python

"""
This is a mock FPU which manages a bit of
internal state

"""

from __future__ import print_function

import numpy as np

MAX_ENTRIES = 256

class FPU:
    IDXA = 0
    IDXB = 1
    
    def __init__(self, fpu_id):
        self.fpu_id = fpu_id
        self.alpha_steps = fpu_id
        self.beta_steps = 10000 + fpu_id * 100
        self.moving = False
        self.wave_ready = false
        self.nwave_entries = 0
        self.steps = np.zeros((256, 2), detype=np.int)
        self.pause = np.zeros((256, 2), detype=np.bool)
        self.clockwise = np.zeros((256, 2), detype=np.bool)


    def addStep(self, first, last,
                asteps, apause, aclockwise,
                bsteps, bpause, blockwise):
        if self.moving:
            raise RuntimeError("FPU is moving")
        
        if first:
            self.nwave_entries = 0

        self.wave_ready = bool(last)
            
        n = self.nwave_entries

        if n  >= MAX_WAVE_ENTRIES:
            raise IndexError
        
        self.steps[n, IDXA] = asteps
        self.steps[n, IDXB] = bsteps
        
        self.pause[n, IDXA] = apause
        self.pause[n, IDXB] = bpause
        
        self.clockwise[n, IDXA] = aclockwise
        self.clockwise[n, IDXB] = bclockwise

        self.nwave_entries  += 1
        if last:
            self.wave_ready = True
            
        
        

    
        
        

