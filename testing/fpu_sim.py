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

ALPHA_SWITCH_WIDTH = 5

ALPHA_CRASH_MARGIN = 15


ALPHA_ZERO = 0
MIN_ALPHA_ON = int(ALPHA_LIMIT_MIN_DEGREE  * StepsPerDegreeAlpha)
MIN_ALPHA_OFF = int((ALPHA_LIMIT_MIN_DEGREE - ALPHA_SWITCH_WIDTH) * StepsPerDegreeAlpha)
MAX_ALPHA_ON = int(ALPHA_LIMIT_MAX_DEGREE  * StepsPerDegreeAlpha)
MAX_ALPHA_OFF = int((ALPHA_LIMIT_MAX_DEGREE + ALPHA_SWITCH_WIDTH) * StepsPerDegreeAlpha)

MIN_ALPHA_CRASH = int((ALPHA_LIMIT_MIN_DEGREE - ALPHA_SWITCH_WIDTH - ALPHA_CRASH_MARGIN) * StepsPerDegreeAlpha)
MAX_ALPHA_CRASH = int((ALPHA_LIMIT_MAX_DEGREE + ALPHA_SWITCH_WIDTH + ALPHA_CRASH_MARGIN) * StepsPerDegreeAlpha)


MIN_BETA = int(BETA_LIMIT_MIN_DEGREE * StepsPerDegreeBeta)
MAX_BETA = int(BETA_LIMIT_MAX_DEGREE * StepsPerDegreeBeta)

BETA_CRASH_MARGIN = 5
MIN_BETA_CRASH = int((BETA_LIMIT_MIN_DEGREE - BETA_CRASH_MARGIN) * StepsPerDegreeBeta)
MAX_BETA_CRASH = int((BETA_LIMIT_MAX_DEGREE + BETA_CRASH_MARGIN) * StepsPerDegreeBeta)


# E_REQUEST_DIRECTION

REQD_ANTI_CLOCKWISE = 0
REQD_CLOCKWISE      = 1

def printtime():
    print(time.strftime("%a, %d %b %Y %H:%M:%S +0000 (%Z)", time.gmtime()))

    
class LimitBreachException(Exception):
    pass

class BetaCollisionException(Exception):
    pass

class CrashException(Exception):
    pass

class FPU:
    
    def __init__(self, fpu_id, opts):
        self.opts = opts
        self.initialize(fpu_id)
        print("FPU %i initial offset: (%f, %f)" % (fpu_id, opts.alpha_offset, opts.beta_offset))
        self.aoff_steps = int(StepsPerDegreeAlpha * opts.alpha_offset)
        self.boff_steps = int(StepsPerDegreeBeta * opts.beta_offset)

    def initialize(self, fpu_id):
        self.fpu_id = fpu_id
        self.alpha_steps = 0
        self.beta_steps = 0
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

        fw_date = self.opts.fw_date
        
        print("firmware date=",fw_date)

        version_tuple = self.opts.fw_version
            
        self.firmware_major = version_tuple[0]
        self.firmware_minor = version_tuple[1]
        self.firmware_patch = version_tuple[2]
        self.firmware_year = fw_date[0]
        self.firmware_month = fw_date[1]
        self.firmware_day = fw_date[2]

    def getRegister(self, register_address):
        if register_address == 0:
            return self.firmware_major
        elif register_address == 1:
            return self.firmware_minor
        elif register_address == 2:
            return self.firmware_patch
        elif register_address == 3:
            return self.firmware_year
        elif register_address == 4:
            return self.firmware_month
        elif register_address == 5:
            return self.firmware_day
        else:
            return 0xff

    def resetFPU(self, fpu_id, sleep):
        printtime()
        print("resetting FPU #%i..." % fpu_id)
        dtime_mu = 0.1
        dtime_sigma = 1.0
        dtime_sec = min(max(random.gauss(dtime_mu, dtime_sigma), 0.1), 2)
        sleep(dtime_sec)
        alpha_steps = self.alpha_steps
        beta_steps = self.beta_steps
        self.initialize(fpu_id)
        # if the FPU is not at datum, enlarge the step mismatch re physical position
        self.aoff_steps += alpha_steps
        self.boff_steps += beta_steps
        print("resetting FPU #%i... ready" % fpu_id)

        
    def repeatMotion(self, fpu_id):
        print("repeating wavetable of FPU #%i..." % fpu_id)
        if not (self.wave_valid):
            raise RuntimeError("wavetable not valid")
        self.move_forward = True
        self.wave_ready = True

    def setUStepLevel(self, ustep_level):
        print("setting UStepLevel for fpu # %i to %i" % (self.fpu_id, ustep_level))
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

    def alpha_switch_on(self, asteps):
        alpha_steps = asteps + self.aoff_steps
        return ( ((alpha_steps > MIN_ALPHA_OFF)
                  and (alpha_steps < MIN_ALPHA_ON))
                 or ( ((alpha_steps > MAX_ALPHA_ON)
                  and (alpha_steps < MAX_ALPHA_OFF))))
                

    def move_alpha(self, newalpha, limit_callback):

        old_switch = self.alpha_switch_on(self.alpha_steps)
        new_switch = self.alpha_switch_on(newalpha)

        out_direction = (newalpha < MIN_ALPHA_ON) or (newalpha > MAX_ALPHA_ON)
        alpha_offset = self.aoff_steps

        self.alpha_steps = newalpha
        
        if (not new_switch) and old_switch:
            self.alpha_limit_breach = True
            limit_callback(self)
        
            if (newalpha + alpha_offset) < MIN_ALPHA_OFF:
                self.alpha_steps = MIN_ALPHA_OFF
                raise LimitBreachException("An alpha min limit breach occured")        
            elif (newalpha + alpha_offset) > MAX_ALPHA_OFF:
                self.alpha_steps = MAX_ALPHA_OFF
                raise LimitBreachException("An alpha max limit breach occured")        
        else:
            if (newalpha + alpha_offset) < MIN_ALPHA_CRASH:
                raise CrashException("An min alpha crash occured")        
            if (newalpha + alpha_offset) > MAX_ALPHA_CRASH:
                print("newalpha=",newalpha,
                      ", alpha_soffset=", alpha_offset,
                      ", MAX_ALPHA_CRASH=", MAX_ALPHA_CRASH)
                raise CrashException("An max alpha crash occured")        
                
            

    def move_beta(self, newbeta, collision_callback):
        beta_offset = self.boff_steps
        self.beta_steps = newbeta
        if ((newbeta + beta_offset) < MIN_BETA) and self.collision_protection_active :
            self.beta_steps = MIN_BETA
            self.is_collided = True
            collision_callback(self)
            raise BetaCollisionException("a beta arm collision was detected")
        elif ((newbeta + beta_offset) > MAX_BETA) and self.collision_protection_active:
            self.is_collided = True
            self.beta_steps = MAX_BETA
            collision_callback(self)
            raise BetaCollisionException("a beta arm collision was detected")
        else:
            if (newbeta + beta_offset) < MIN_BETA_CRASH:
                raise CrashException("An min beta crash occured")        
            if (newbeta + beta_offset) > MAX_BETA_CRASH:
                raise CrashException("An max beta crash occured")        
                
        
    def findDatum(self, sleep, limit_callback, collision_callback,
                  skip_alpha=False, skip_beta=False,
                  auto_datum=False, anti_clockwise=False):
        
        printtime()
        alpha_speed = -120.0 # steps per time interval
        beta_speed = -120.0 # steps per interval
        
        if auto_datum:
            if self.beta_steps < 0:
                anti_clockwise = True
            else:
                anti_clockwise = False
                
        if anti_clockwise:
            beta_sign = -1
        else:
            beta_sign = 1
            
        print(("FPU #%i: searching datum, skip_alpha=%r,skip_beta=%r, "
               + "auto_datum=%r, anti_clockwise=%r, beta sign = %i") % (
                   self.fpu_id, skip_alpha, skip_beta, auto_datum, anti_clockwise,
                   beta_sign))
        
        def random_deviation():                
            deviation_mu = 0
            deviation_sigma = 10
            return min(max(random.gauss(deviation_mu, deviation_sigma), -15), 15)
        
        self.alpha_deviation = int(random_deviation())
        
        self.beta_deviation = int(random_deviation())

        #self.alpha_steps += self.alpha_deviation
        #self.beta_steps += self.beta_deviation
                
        print("FPU #%i: findDatum is now at (%i, %i) steps\n" % (self.fpu_id, self.alpha_steps, self.beta_steps))
            
        wait_interval_sec = 0.1

        beta_offset = self.boff_steps
        alpha_offset = self.aoff_steps
        last_beta_angle = self.beta_steps + beta_offset
        last_alpha_angle = self.alpha_steps + alpha_offset

        beta_crossed_zero = False
        alpha_crossed_zero = False
        while True:
            # the model here is that crossing physical zero
            # edge-triggers the stop signal for both arms
            
            beta_angle  = self.beta_steps + beta_offset
            beta_zero_crossing= (last_beta_angle * beta_angle) <= 0
            beta_crossed_zero = beta_crossed_zero or beta_zero_crossing

            alpha_angle  = self.alpha_steps + alpha_offset
            alpha_zero_crossing= (last_alpha_angle * alpha_angle) <= 0
            alpha_crossed_zero = alpha_crossed_zero or alpha_zero_crossing
            
        
            alpha_ready = alpha_crossed_zero or skip_alpha
            beta_ready = beta_crossed_zero or skip_beta

            #print("alpha ready=%r, beta crossed zero=%r, is crossing zero=%r, ready=%r" % (
            #      alpha_ready, beta_crossed_zero, beta_zero_crossing, beta_ready))
            
            if alpha_ready and beta_ready:
                break

            sleep(wait_interval_sec)
            
            if self.abort_wave:
                print("ABORTING DATUM SEARCH FOR FPU", self.fpu_id);
                self.at_datum = False
                break


            
            try:
                if not alpha_ready:
                    new_alpha = self.alpha_steps + int(alpha_speed)
                    self.move_alpha(new_alpha, limit_callback)
            except LimitBreachException:
                print("Alpha limit breach for FPU  %i" % self.fpu_id)
                break
            
            try:
                if not beta_ready:
                    # this is much simplified: search until crossing the zero point
                    if not beta_crossed_zero:
                        new_beta = self.beta_steps + int(beta_speed * beta_sign)
                        self.move_beta(new_beta, collision_callback)
            except BetaCollisionException:
                print("Beta collision for FPU  %i" % self.fpu_id)
                break

            #print("FPU# %i: skip_alpha=%r, skip_beta=%r, beta_sign=%f" % (self.fpu_id, skip_alpha, skip_beta, beta_sign))
            print("FPU #%i: findDatum is now at (%i, %i) steps" % (self.fpu_id, self.alpha_steps, self.beta_steps))

            

        if not self.abort_wave:

            if not skip_alpha:
                self.alpha_steps = 0
                self.aoff_steps = 0
                if skip_beta:
                    print("FPU #%i: alpha datum reached" % self.fpu_id)
            if not skip_beta:
                self.beta_steps = 0
                self.boff_steps = 0
                if skip_alpha:
                    print("FPU #%i: beta datum reached" % self.fpu_id)
            if not (skip_alpha or skip_beta):
                self.at_datum = True
                print("FPU #%i: datum found" % self.fpu_id)
            else:
                print("FPU #%i: partial datum operation finished" % self.fpu_id)

        printtime()
            

    def abortMotion(self, sleep):
        self.abort_wave = True


    def freeBetaCollision(self, direction):
        UNTANGLE_STEPS = 10
        alpha_offset = self.aoff_steps
        beta_offset = self.boff_steps
        
        self.collision_protection_active = False

        old_beta_steps = self.beta_steps
        
        if direction == REQD_CLOCKWISE:
            self.beta_steps -= UNTANGLE_STEPS
        else:
            self.beta_steps += UNTANGLE_STEPS

        alpha_real_deg =  (self.alpha_steps + alpha_offset) / StepsPerDegreeAlpha
        beta_real_deg =  (self.beta_steps + beta_offset) / StepsPerDegreeBeta
        
        print("freeBetaCollsion: moving FPU # %i from (%i,%i) to (%i, %i) = real (%5.2f,5.2f) deg" % (
            self.fpu_id, self.alpha_steps, old_beta_steps,
            self.alpha_steps, self.beta_steps,
            alpha_real_deg, beta_real_deg))
        
        if ((self.beta_steps + beta_ffset) >= MAX_BETA) or (
                (self.beta_steps + beta_offset) <= MIN_BETA):
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


        beta_offset = self.boff_steps
        
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
            new_alpha = self.alpha_steps + delta_alpha
            if self.clockwise[n, IDXB]:
                beta_sign = -1
            else:
                beta_sign = 1
            delta_beta = wt_sign * beta_sign * self.steps[n,IDXB]
            new_beta = self.beta_steps + delta_beta

            alpha_offset = self.aoff_steps
            beta_offset = self.boff_steps
            alpha_real_deg =  (new_alpha + alpha_offset) / StepsPerDegreeAlpha
            beta_real_deg =  (new_beta + beta_offset) / StepsPerDegreeBeta
        
            if self.opts.verbosity > 0:
                print("section %i: moving FPU %i by (%i,%i) to (%i, %i) = real (%5.2f, %5.2f) deg" % (
                    n, self.fpu_id, delta_alpha, delta_beta, new_alpha, new_beta,
                    alpha_real_deg, beta_real_deg))
            
            frame_time = 0.25
            sleep(frame_time)
            if self.abort_wave:
                break

            if (delta_beta > 0) or (delta_alpha > 0):
                self.at_datum = False

            if self.ustep_level == 8:
                if random.uniform(0, 100) < 50:
                    self.step_timing_fault = True
                    break

            try:
                self.move_alpha(new_alpha, limit_callback)
            except LimitBreachException:
                print("Alpha limit breach for FPU  %i" % self.fpu_id)
                break
            
            try:
                self.move_beta(new_beta, collision_callback)
            except BetaCollisionException:
                print("Beta collision for FPU  %i" % self.fpu_id)
                break
            
        printtime()
        if self.abort_wave:
            print("FPU %i, section %i: MOVEMENT ABORTED at (%i, %i)" % (self.fpu_id, section,
                                                            new_alpha, new_beta))
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
                                                             new_alpha, new_beta))
        self.running_wave = False
        self.wave_ready = False
            
            

def init_FPUGrid(options, num_fpus):        
    FPUGrid[:] = [FPU(i, options) for i in range(num_fpus) ]
    
        
        

