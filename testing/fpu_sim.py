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

MAX_ALPHA = 41000
MIN_ALPHA = -1000

MAX_BETA = 30000
MIN_BETA = -30000

# E_REQUEST_DIRECTION

REQD_ANTI_CLOCKWISE = 0
REQD_CLOCKWISE      = 1


class FPU:
    
    def __init__(self, fpu_id):
        self.initialize(fpu_id)

    def initialize(self, fpu_id):
        self.fpu_id = fpu_id
        self.alpha_steps = fpu_id - 50
        self.beta_steps = fpu_id * 10 - 50
        self.alpha_deviation = 0
        self.beta_deviation = 0
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
        self.wave_valid = False
        self.collision_protection_active = True        

    def resetFPU(self, fpu_id, sleep):
        print("resetting FPU #%i..." % fpu_id)
        dtime_mu = 0.1
        dtime_sigma = 1.0
        dtime_sec = min(max(random.gauss(dtime_mu, dtime_sigma), 0.1), 2)
        sleep(dtime_sec)
        self.initialize(fpu_id)
        print("resetting FPU #%i... ready" % fpu_id)

        
    def repeatMotion(self, fpu_id):
        print("repeating wavetable of FPU #%i..." % fpu_id)
        if not (self.wave_valid):
            raise RuntimeError("wavetable not valid")
        self.move_forward = True
        self.wave_ready = True
        
        
    def reverseMotion(self, fpu_id):
        print("reversing wavetable of FPU #%i..." % fpu_id)
        if not (self.wave_valid):
            raise RuntimeError("wavetable not valid")
        self.move_forward = False
        self.wave_ready = True
        
        

    def addStep(self, first, last,

                asteps, apause, aclockwise,
                bsteps, bpause, bclockwise, verbose=0):
        if self.running_wave:
            raise RuntimeError("FPU is moving")
        
        if first:
            self.nwave_entries = 0
            self.wave_ready = False
            self.wave_valid = False
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
            self.wave_valid = True
            n = self.nwave_entries
            if verbose:
                print("fpu #%i: wavetable ready, n=%i, content=%s" % (
                    
                    self.fpu_id, n, (self.steps[0:n], self.pause[0:n], self.clockwise[0:n])))
            else:
                print("fpu #%i: wavetable ready (%i steps)" % (self.fpu_id, n))
        
        
    def findDatum(self, sleep):
        datum_op_duration_mu = 1
        datum_op_duration_sigma = 2
        datum_op_duration_sec = min(max(random.gauss(datum_op_duration_mu, datum_op_duration_sigma), 0), 15)
        wait_interval_sec = 0.1
        while datum_op_duration_sec > 0:
            sleep_time = min(datum_op_duration_sec, wait_interval_sec)
            sleep(sleep_time)
            datum_op_duration_sec -= sleep_time
            
            if self.abort_wave:
                print("ABORTING DATUM SEARCH FOR FPU", self.fpu_id);
                self.at_datum = True
                break

        if not self.abort_wave:
            def random_deviation():                
                deviation_mu = 0
                deviation_sigma = 10
                return min(max(random.gauss(deviation_mu, deviation_sigma), -15), 15)
            
            self.alpha_deviation = int(random_deviation())
            self.beta_deviation = int(random_deviation())
            self.alpha_steps = 0
            self.beta_steps = 0
            self.at_datum = True
            

    def abortMotion(self, sleep):
        self.abort_wave = True


    def freeBetaCollision(self, direction):
        UNTANGLE_STEPS = 10
        
        self.collision_protection_active = False

        old_beta_steps = self.beta_steps
        
        if direction == REQD_CLOCKWISE:
            self.beta_steps -= UNTANGLE_STEPS
        else:
            self.beta_steps += UNTANGLE_STEPS

        print("freeBetaCollsion: moving FPU # %i from (%i,%i) to (%i, %i)" % (
            self.fpu_id, self.alpha_steps, old_beta_steps,
            self.alpha_steps, self.beta_steps))
        
        if (self.beta_steps >= MAX_BETA) or (self.beta_steps <= MIN_BETA):
            self.is_collided = True
            print("FPU #%i: collision ongoing" % self.fpu_id)
        else:
            self.is_collided = False
            print("FPU #%i: collision resolved" % self.fpu_id)


    def enableBetaCollisionProtection(self):
        self.collision_protection_active = True
            

        
    def executeMotion(self, sleep, limit_callback, collision_callback):
        if self.running_wave :
            raise RuntimeError("FPU is already moving")
        
        if not (self.wave_ready):
            raise RuntimeError("wavetable not ready")

        self.running_wave = True

        if self.move_forward:
            wt_sign = 1
        else:
            wt_sign = -1
        
        for k in range(self.nwave_entries):
            
            if self.abort_wave:
                # flag was set from abortMotion command
                break
            
            if self.move_forward:
                n = k
            else:
                n = self.nwave_entries - k - 1
            if self.clockwise[n, IDXA]:
                alpha_sign = -1
            else:
                alpha_sign = 1
            delta_alpha = wt_sign * alpha_sign * self.steps[n,IDXA]
            newalpha = self.alpha_steps + delta_alpha
            if self.clockwise[n, IDXB]:
                beta_sign = -1
            else:
                beta_sign = 1
            delta_beta = wt_sign * beta_sign * self.steps[n,IDXB]
            newbeta = self.beta_steps + delta_beta
            
            print("step %i: moving FPU %i by (%i,%i) to (%i, %i)" % (
                n, self.fpu_id, delta_alpha, delta_beta, newalpha, newbeta))
            
            frame_time = 0.25
            sleep(frame_time)
            if self.abort_wave:
                break
            
            if newalpha < MIN_ALPHA:
                self.alpha_steps = MIN_ALPHA
                self.alpha_limit_breach = True
                limit_callback(self)
                break
            elif newalpha > MAX_ALPHA:
                self.alpha_steps = MAX_ALPHA
                self.alpha_limit_breach = True
                limit_callback(self)
                break
            else:
                self.alpha_steps = newalpha
                
            if (newbeta < MIN_BETA) and self.collision_protection_active :
                self.beta_steps = MIN_BETA
                self.is_collided = True
                collision_callback(self)
                break
            elif (newbeta > MAX_BETA) and self.collision_protection_active:
                self.is_collided = True
                self.beta_steps = MAX_BETA
                collision_callback(self)
                break
            else:
                self.beta_steps = newbeta

        if self.abort_wave:
            print("FPU %i: MOVEMENT ABORTED at (%i, %i)" % (self.fpu_id,
                                                            newalpha, newbeta))
        elif self.alpha_limit_breach:
            print("FPU %i: limit switch breach, movement cancelled at (%i, %i)" % (self.fpu_id,
                                                                                   self.alpha_steps, self.beta_steps))
        elif self.is_collided:
            print("FPU %i: beta_collision, movement cancelled at (%i, %i)" % (self.fpu_id,
                                                                              self.alpha_steps, self.beta_steps))
        else:
            print("FPU %i: movement finished at (%i, %i)" % (self.fpu_id,
                                                             newalpha, newbeta))
        self.running_wave = False
        self.wave_ready = False
            
            
            
    
        
        

