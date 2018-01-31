#!/usr/bin/python

"""
This is a mock FPU which manages a bit of
internal state

"""

from __future__ import print_function

import numpy as np
import random

MAX_WAVE_ENTRIES = 256

IDXA = 0
IDXB = 1

class FPU:
    
    def __init__(self, fpu_id):
        self.initialize(fpu_id)

    def initialize(self, fpu_id):
        self.fpu_id = fpu_id
        self.alpha_steps = fpu_id - 50
        self.beta_steps = fpu_id * 10 - 50
        self.nwave_entries = 0
        self.steps = np.zeros((256, 2), dtype=np.int)
        self.pause = np.zeros((256, 2), dtype=np.bool)
        self.clockwise = np.zeros((256, 2), dtype=np.bool)
        self.is_collided = False
        self.alpha_limit_breach = False
        self.at_datum = False
        self.wave_ready = False
        self.move_forward = True
        self.running_wave = False
        self.abort_wave = False
        

    def resetFPU(self, fpu_id, sleep):
        print("resetting FPU #%i..." % fpu_id)
        dtime_mu = 0.1
        dtime_sigma = 1.0
        dtime_sec = min(max(random.gauss(dtime_mu, dtime_sigma), 0.1), 2)
        sleep(dtime_sec)
        self.initialize(fpu_id)
        print("resetting FPU #%i... ready" % fpu_id)
        
        

    def addStep(self, first, last,
                asteps, apause, aclockwise,
                bsteps, bpause, bclockwise):
        if self.running_wave:
            raise RuntimeError("FPU is moving")
        
        if first:
            self.nwave_entries = 0
            self.wave_ready = False
            self.move_forward = True
            # clear abort status flag
            self.abort_wave = False 
            
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
            n = self.nwave_entries
            print("fpu #%i: wavetable ready, n=%i, content=%s" % (
                
                self.fpu_id, n, (self.steps[0:n], self.pause[0:n], self.clockwise[0:n])))
            
        
        
    def findDatum(self, sleep):
        dtime_mu = 1
        dtime_sigma = 2
        dtime_sec = min(max(random.gauss(dtime_mu, dtime_sigma), 0), 15)
        sleep(dtime_sec)
        self.alpha_steps = 0
        self.beta_steps = 0
        self.at_datum = True

    def executeMotion(self, sleep):
        if self.running_wave :
            raise RuntimeError("FPU is already moving")
        
        if not (self.wave_ready):
            raise RuntimeError("wavetable not ready")

        self.running_wave = True
        
        for k in range(self.nwave_entries):
            if self.move_forward:
                n = k
            else:
                n = self.nwave_entries - k - 1
            if self.clockwise[n, IDXA]:
                alpha_sign = -1
            else:
                alpha_sign = 1
            delta_alpha = alpha_sign * self.steps[n,IDXA]
            newalpha = self.alpha_steps + delta_alpha
            if self.clockwise[n, IDXB]:
                beta_sign = -1
            else:
                beta_sign = 1
            delta_beta = beta_sign * self.steps[n,IDXB]
            newbeta = self.beta_steps + delta_beta
            
            print("step %i: moving FPU %i by (%i,%i) to (%i, %i)" % (
                n, self.fpu_id, delta_alpha, delta_beta, newalpha, newbeta))
            
            frame_time = 0.25
            sleep(frame_time)
            if self.abort_wave:
                break
            self.alpha_steps = newalpha
            self.beta_steps = newbeta

        if self.abort_wave:
            print("FPU %i: MOVEMENT ABORTED at (%i, %i)" % (self.fpu_id,
                                                            newalpha, newbeta))
        else:
            print("FPU %i: movement finished at (%i, %i)" % (self.fpu_id,
                                                             newalpha, newbeta))
        self.running_wave = False
            
            
            
    
        
        

