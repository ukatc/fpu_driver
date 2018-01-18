#!/usr/bin/python

"""
This is a mock FPU which manages a bit of
internal state

"""

from __future__ import print_function

import numpy as np
import random

MAX_ENTRIES = 256

class FPU:
    IDXA = 0
    IDXB = 1
    
    def __init__(self, fpu_id):
        self.fpu_id = fpu_id
        self.alpha_steps = fpu_id
        self.beta_steps = fpu_id * 1000
        self.moving = False
        self.wave_ready = False
        self.nwave_entries = 0
        self.steps = np.zeros((256, 2), dtype=np.int)
        self.pause = np.zeros((256, 2), dtype=np.bool)
        self.clockwise = np.zeros((256, 2), dtype=np.bool)
        self.is_collided = False
        self.alpha_limit_breach = False
        self.at_datum = False
        


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
            
        
        
    def findDatum(self, sleep):
        dtime_mu = 0.1
        dtime_sigma = 0.2
        dtime_sec = min(max(random.gauss(dtime_mu, dtime_sigma), 0), 5)
        #sleep(dtime_sec)
        self.alpha_steps = 0
        self.beta_steps = 0
        self.at_datum = True

    
        
        

