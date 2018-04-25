#!/usr/bin/python

"""
This is a mock FPU which manages a bit of
internal state

"""

from __future__ import print_function

import numpy as np
import random
import time

MAX_WAVE_ENTRIES = 128

IDXA = 0
IDXB = 1

FPUGrid = []

# These are parameters from the instrument control system
# They do NOT match the origin of the FPU step counter

# INS_POS_LOW1      = -180.000  # Lower travel limit of positioner alpha arm (deg)
# INS_POS_HIGH1     =  180.000  # Upper travel limit of positioner alpha arm (deg)
# INS_POS_LOW2      = -180.000  # Lower travel limit of positioner beta arm (deg)
# INS_POS_HIGH2     =  150.000  # Upper travel limit of positioner beta arm (deg)

ALPHA_MIN_DEGREE = 0
ALPHA_MAX_DEGREE = 360
BETA_MIN_DEGREE = -180
BETA_MAX_DEGREE = 130


ALPHA_LIMIT_MIN_DEGREE = ALPHA_MIN_DEGREE -5
ALPHA_LIMIT_MAX_DEGREE = ALPHA_MAX_DEGREE + 5
BETA_LIMIT_MIN_DEGREE  = BETA_MIN_DEGREE - 5
BETA_LIMIT_MAX_DEGREE  = BETA_MAX_DEGREE + 5

AlphaGearRatio 	= 2050.175633 # actual gear ratio
BetaGearRatio 	= 1517.662482 # actual gear ratio


# There are 20 steps per revolution on the non-geared side, so:
StepsPerRevolution = 20.0
DegreePerRevolution = 360.0

# Note that these numbers must not be confounded with actual calibrated values!

StepsPerDegreeAlpha = (StepsPerRevolution * AlphaGearRatio) / DegreePerRevolution
StepsPerDegreeBeta = (StepsPerRevolution * BetaGearRatio) / DegreePerRevolution

MIN_ALPHA = int(ALPHA_LIMIT_MIN_DEGREE * StepsPerDegreeAlpha)
MAX_ALPHA = int(ALPHA_LIMIT_MAX_DEGREE * StepsPerDegreeAlpha)

MIN_BETA = int(BETA_LIMIT_MIN_DEGREE * StepsPerDegreeBeta)
MAX_BETA = int(BETA_LIMIT_MAX_DEGREE * StepsPerDegreeBeta)

# E_REQUEST_DIRECTION

REQD_ANTI_CLOCKWISE = 0
REQD_CLOCKWISE      = 1

def printtime():
    print(time.strftime("%a, %d %b %Y %H:%M:%S +0000 (%Z)", time.gmtime()))


class FPU:
    
    def __init__(self, fpu_id, opts):
        self.opts = opts
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
        self.ustep_level = 1
        self.step_timing_fault = False

    def resetFPU(self, fpu_id, sleep):
        printtime()
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

    def setUStepLevel(self, ustep_level):
        print("setting UStepLevel=", ustep_level)
        self.ustep_level = ustep_level
        
    def reverseMotion(self, fpu_id):
        print("reversing wavetable of FPU #%i..." % fpu_id)
        if not (self.wave_valid):
            raise RuntimeError("wavetable not valid")
        self.move_forward = False
        self.wave_ready = True
        
        

    def addStep(self, first, last,

                asteps, apause, aclockwise,
                bsteps, bpause, bclockwise):
        
        verbose = self.opts.verbosity > 4
        
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
                print("fpu #%i: wavetable ready (%i sections)" % (self.fpu_id, n))
        
        
    def findDatum(self, sleep):
        printtime()
        datum_op_duration_mu = 1
        datum_op_duration_sigma = 2
        datum_op_duration_sec = min(max(random.gauss(datum_op_duration_mu, datum_op_duration_sigma), 0), 5)
        print("FPU #%i: searching datum" % self.fpu_id)
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
        print("FPU #%i: datum found" % self.fpu_id)
        printtime()
            

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
        printtime()
        if self.running_wave :
            raise RuntimeError("FPU is already moving")
        
        if not (self.wave_ready):
            raise RuntimeError("wavetable not ready")


        self.running_wave = True
        self.step_timing_fault = False
        
        if self.move_forward:
            wt_sign = 1
        else:
            wt_sign = -1

        #simulate between 0 and 10 ms of latency
        latency_secs = random.uniform(0, 10) / 1000.
        sleep(latency_secs)
        
        section = -1
        for k in range(self.nwave_entries):
            section = k
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
            
            print("section %i: moving FPU %i by (%i,%i) to (%i, %i)" % (
                n, self.fpu_id, delta_alpha, delta_beta, newalpha, newbeta))
            
            frame_time = 0.25
            sleep(frame_time)
            if self.abort_wave:
                break

            if self.ustep_level == 8:
                if random.uniform(0, 100) < 50:
                    self.step_timing_fault = True
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

        printtime()
        if self.abort_wave:
            print("FPU %i, section %i: MOVEMENT ABORTED at (%i, %i)" % (self.fpu_id, section,
                                                            newalpha, newbeta))
        elif self.alpha_limit_breach:
            print("FPU %i, section %i: limit switch breach, movement cancelled at (%i, %i)" % (self.fpu_id, section,
                                                                                   self.alpha_steps, self.beta_steps))
        elif self.is_collided:
            print("FPU %i, section %i: beta_collision, movement cancelled at (%i, %i)" % (self.fpu_id, section,
                                                                              self.alpha_steps, self.beta_steps))
        elif self.step_timing_fault:
            print("FPU %i, section %i: step timing error, movement cancelled at (%i, %i)" % (self.fpu_id, section,
                                                                                self.alpha_steps, self.beta_steps))
        else:
            print("FPU %i, section %i: movement finished at (%i, %i)" % (self.fpu_id, section,
                                                             newalpha, newbeta))
        self.running_wave = False
        self.wave_ready = False
            
            

def init_FPUGrid(options, num_fpus):        
    FPUGrid[:] = [FPU(i, options) for i in range(num_fpus) ]
    
        
        

