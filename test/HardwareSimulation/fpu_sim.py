#!/usr/bin/python

"""
This is a mock FPU which manages a bit of
internal state

"""

from __future__ import print_function

import numpy as np
import random
import time
import signal

import inspect
#import hashlib
import binascii

from protocol_constants import *


IDXA = 0
IDXB = 1

FPUGrid = []

# These are parameters from the instrument control system
# They do NOT match the origin of the FPU step counter

# INS_POS_LOW1      = -180.000  # Lower travel limit of positioner alpha arm (deg)
# INS_POS_HIGH1     =  180.000  # Upper travel limit of positioner alpha arm (deg)
# INS_POS_LOW2      = -180.000  # Lower travel limit of positioner beta arm (deg)
# INS_POS_HIGH2     =  150.000  # Upper travel limit of positioner beta arm (deg)

# offset between displayed angles (which are always
# instrument coordinates) and angle relative to datum positions.

ALPHA_DATUM_OFFSET=-180.0
BETA_DATUM_OFFSET=0.0

# range allowed by software protection (copied from fpu_constants.py)
ALPHA_MIN_DEGREE = -180.2
ALPHA_MAX_DEGREE = +159.0

ALPHA_MIN_HARDSTOP_DEGREE = -183.2
ALPHA_MAX_HARDSTOP_DEGREE = +181.8

BETA_MIN_DEGREE = -179.3
BETA_MAX_DEGREE = 140.0

BETA_MIN_HWPROT_DEGREE = -181.0
BETA_MAX_HWPROT_DEGREE = 145.0




# The alpha RDEGREE values below are relative from the datum switch
# (datum=zero).  (For displayed alpha angles, the conventional angle
# at datum is added.)
ALPHA_MIN_DEGREE = 0
ALPHA_MAX_DEGREE = -(-180.0) + 170.0
BETA_MIN_DEGREE = -179.3
BETA_MAX_DEGREE = 150.3
BETA_DATUM_SWITCH_MAX_DEGREE = -0.2
BETA_DATUM_SWITCH_MIN_DEGREE = -5


ALPHA_BUFFER = 0.2
ALPHA_MIN_RDEGREE = ALPHA_MIN_DEGREE - ALPHA_DATUM_OFFSET + ALPHA_BUFFER
ALPHA_MAX_RDEGREE = ALPHA_MAX_DEGREE - ALPHA_DATUM_OFFSET - ALPHA_BUFFER
BETA_DATUM_SWITCH_MAX_RDEGREE = -0.2
BETA_DATUM_SWITCH_MIN_RDEGREE = -5

ALPHA_LIMIT_MIN_RDEGREE = ALPHA_MIN_DEGREE - ALPHA_DATUM_OFFSET # this is the actual relative datum switch point
ALPHA_LIMIT_MAX_RDEGREE = ALPHA_MAX_DEGREE - ALPHA_DATUM_OFFSET

BETA_BUFFER = 0.2
BETA_LIMIT_MIN_RDEGREE  = BETA_MIN_DEGREE - BETA_BUFFER
BETA_LIMIT_MAX_RDEGREE  = BETA_MAX_DEGREE + BETA_BUFFER

assert(ALPHA_MIN_RDEGREE < ALPHA_MAX_RDEGREE)
assert(ALPHA_LIMIT_MIN_RDEGREE < ALPHA_MIN_RDEGREE)
assert(ALPHA_MAX_RDEGREE < ALPHA_LIMIT_MAX_RDEGREE)

assert(BETA_LIMIT_MIN_RDEGREE < BETA_MIN_DEGREE)
assert(BETA_MIN_DEGREE < BETA_MAX_DEGREE)
assert(BETA_MAX_DEGREE < BETA_LIMIT_MAX_RDEGREE)

AlphaGearRatio 	= 2050.175633 # actual gear ratio
BetaGearRatio 	= 1517.662482 # actual gear ratio

# There are 20 steps per revolution on the non-geared side, so:
StepsPerRevolution = 20.0
DegreePerRevolution = 360.0

# Note that these numbers must not be confounded with actual calibrated values!

StepsPerDegreeAlpha = (StepsPerRevolution * AlphaGearRatio) / DegreePerRevolution
StepsPerDegreeBeta = (StepsPerRevolution * BetaGearRatio) / DegreePerRevolution

ALPHA_SWITCH_WIDTH = 2.0



# step counts where switches go on and off
MIN_ALPHA_ON = int(ALPHA_LIMIT_MIN_RDEGREE  * StepsPerDegreeAlpha)
MIN_ALPHA_OFF = int((ALPHA_LIMIT_MIN_RDEGREE - ALPHA_SWITCH_WIDTH) * StepsPerDegreeAlpha)

MAX_ALPHA_ON = int(ALPHA_LIMIT_MAX_RDEGREE  * StepsPerDegreeAlpha)
MAX_ALPHA_OFF = int((ALPHA_LIMIT_MAX_RDEGREE + ALPHA_SWITCH_WIDTH) * StepsPerDegreeAlpha)

# point where collision detection is triggered by reeaching beta range limit
MIN_BETA = int(BETA_LIMIT_MIN_RDEGREE * StepsPerDegreeBeta)
MAX_BETA = int(BETA_LIMIT_MAX_RDEGREE * StepsPerDegreeBeta)

# step counts where alpha hardware limits kick in
MIN_ALPHA_CRASH = int((ALPHA_MIN_HARDSTOP_DEGREE - ALPHA_DATUM_OFFSET) * StepsPerDegreeAlpha)
MAX_ALPHA_CRASH = int((ALPHA_MAX_HARDSTOP_DEGREE - ALPHA_DATUM_OFFSET) * StepsPerDegreeAlpha)

# verification rig test simulation parameters
BETA_CTEST_DEGREE = 70 # this is defined by the rig setup
BETA_CTEST = int(BETA_CTEST_DEGREE * StepsPerDegreeBeta)

# point where beta arm would crash
MIN_BETA_CRASH = int(BETA_MIN_HWPROT_DEGREE * StepsPerDegreeBeta)
MAX_BETA_CRASH = int(BETA_MAX_HWPROT_DEGREE * StepsPerDegreeBeta)


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
        print("FPU %i initial offset: (%f, %f)" % (fpu_id, opts.alpha_start, opts.beta_start))
        self.aoff_steps = int(StepsPerDegreeAlpha * opts.alpha_start)
        self.boff_steps = int(StepsPerDegreeBeta * opts.beta_start)
        fname = ".sn/_FPU-%04i.sn" % self.fpu_id
        try:
            with open(fname,"r") as f:
                self.serial_number = f.readline().strip("\n")
                print("FPU %i: serial number set to %r" % (self.fpu_id, self.serial_number))
        except:
            self.serial_number = "UNKWN"

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
        self.was_initialized = False
        self.wave_ready = False
        self.wave_reversed = False
        self.running_wave = False
        self.abort_wave = False
        self.wave_valid = False
        self.collision_protection_active = True
        self.ustep_level = 1
        self.step_timing_fault = False
        self.alpha_switch_direction = 0
        self.datum_timeout = False
        self.can_overflow = False
        self.alpha_datum_active = False
        self.beta_datum_active = False
        self.alpha_last_direction = 0
        self.beta_last_direction = 0
        self.fpu_locked = False
        self.state = FPST_UNINITIALIZED



        fw_date = self.opts.fw_date

        print("firmware date=",fw_date)

        version_tuple = self.opts.fw_version

        self.firmware_major = version_tuple[0]
        self.firmware_minor = version_tuple[1]
        self.firmware_patch = version_tuple[2]
        self.firmware_year = fw_date[0]
        self.firmware_month = fw_date[1]
        self.firmware_day = fw_date[2]
        self.fw_version_address_offset = 0x61
        if version_tuple < (1,4,4):
            self.fw_version_address_offset = 0x0

    def getRegister(self, register_address):
        fwa_offset = self.fw_version_address_offset
        if register_address == 0 + fwa_offset:
            return self.firmware_major
        elif register_address == 1 + fwa_offset:
            return self.firmware_minor
        elif register_address == 2 + fwa_offset:
             return self.firmware_patch
        elif register_address == 3 + fwa_offset:
            return self.firmware_year
        elif register_address == 4 + fwa_offset:
            return self.firmware_month
        elif register_address == 5 + fwa_offset:
            return self.firmware_day
        elif register_address == 0x0060:
            if self.opts.fw_version < (1,3,2):
                return 0
            else:
                alpha_switch_bit = self.alpha_switch_on(self.alpha_steps)
                beta_angle = (self.beta_steps + self.boff_steps) / StepsPerDegreeBeta
                beta_switch_bit = ( beta_angle >= BETA_DATUM_SWITCH_MIN_RDEGREE) and (
                    beta_angle <= BETA_DATUM_SWITCH_MAX_RDEGREE)
                byte = 0
                if alpha_switch_bit:
                    byte |= 1
                if beta_switch_bit:
                    byte |= 2
                return byte
        else:
            return 0xff

    def writeSerialNumber(self, serial_number):
        print("FPU #%i: writing serial number '%s' to NVRAM" % (self.fpu_id, serial_number))
        assert(len(serial_number) <= LEN_SERIAL_NUMBER)
        self.serial_number = serial_number
        fname = ".sn/_FPU-%04i.sn" % self.fpu_id
        with open(fname,"w") as f:
            f.write("%s\n" % self.serial_number)



    def readSerialNumber(self):
        print("FPU #%i: reading serial number '%s' from NVRAM" % (self.fpu_id, self.serial_number))
        return self.serial_number

    def getFirmwareVersion(self):
        t =  (self.firmware_major,
              self.firmware_minor,
              self.firmware_patch,
              self.firmware_year,
              self.firmware_month,
              self.firmware_day)

        print("FPU #%i: reading firmware version '%s' from NVRAM" % (self.fpu_id, t))
        return t



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


    def resetStepCounter(self, new_alpha_steps, new_beta_steps):

        if self.state == FPST_LOCKED:
            return MCE_NOTIFY_COMMAND_IGNORED

        # don't allow counter reset when moving
        if self.state in [ FPST_MOVING, FPST_DATUM_SEARCH]:
            return MCE_ERR_INVALID_COMMAND

        printtime()
        print("resetting FPU #%i stepcounters to (%i, %i)" % (self.fpu_id, new_alpha_steps, new_beta_steps))
        old_alpha_steps = self.alpha_steps
        old_beta_steps = self.beta_steps
        self.alpha_steps = new_alpha_steps
        self.beta_steps = new_beta_steps
        # enlarge the modelled step mismatch re physical position
        self.aoff_steps += (old_alpha_steps - new_alpha_steps)
        self.boff_steps += (old_beta_steps - new_beta_steps)
        # we set initialized to "false" because the values are no more
        # defined by the datum position
        self.was_initialized = False

        return MCE_FPU_OK

    def repeatMotion(self, fpu_id):
        if self.state in [ FPST_READY_FORWARD, FPST_LOCKED]:
            return MCE_NOTIFY_COMMAND_IGNORED

        if self.state not in [FPST_READY_FORWARD, FPST_READY_REVERSE, FPST_RESTING]:
            return MCE_ERR_INVALID_COMMAND


        print("reversing wavetable of FPU #%i..." % fpu_id)
        if not (self.wave_valid):
            return MCE_ERR_WAVEFORM_NOT_READY

        self.wave_reversed = False
        self.wave_ready = True
        self.state = FPST_READY_FORWARD

        return MCE_FPU_OK


    def setUStepLevel(self, ustep_level):

        if self.state == FPST_LOCKED:
            return MCE_NOTIFY_COMMAND_IGNORED

        if self.state != FPST_UNINITIALIZED:
            return MCE_NOTIFY_COMMAND_IGNORED

        if ustep_level not in [1,2,4,8]:
            return MCE_ERR_INVALID_PARAMETER

        print("setting UStepLevel for fpu # %i to %i" % (self.fpu_id, ustep_level))
        self.ustep_level = ustep_level

        return MCE_FPU_OK

    def setStepsPerSegment(self, min_steps, max_steps):

        if self.state != FPST_UNINITIALIZED:
            return MCE_NOTIFY_COMMAND_IGNORED

        print("setting stepsPerSegment for fpu # %i to %i .. %i" % (self.fpu_id, min_steps, max_steps))

        # for now, this is without consequences and not used
        self.min_steps = min_steps
        self.max_steps = max_steps

        return MCE_FPU_OK

    def setTicksPerSegment(self, nticks):

        if self.state != FPST_UNINITIALIZED:
            return MCE_NOTIFY_COMMAND_IGNORED

        print("setting ticks per segment for fpu # %i to %i" % (self.fpu_id, nticks))

        # for now, this is without consequences and not used
        self.ticks_per_segment = nticks

        return MCE_FPU_OK


    def reverseMotion(self, fpu_id):
        if self.state in [FPST_READY_REVERSE, FPST_LOCKED ]:
            return MCE_NOTIFY_COMMAND_IGNORED

        if self.state not in [FPST_READY_FORWARD, FPST_READY_REVERSE, FPST_RESTING]:
            return MCE_ERR_INVALID_COMMAND


        print("reversing wavetable of FPU #%i..." % fpu_id)
        if not (self.wave_valid):
            return MCE_ERR_WAVEFORM_NOT_READY

        self.wave_reversed = True
        self.wave_ready = True
        self.state = FPST_READY_REVERSE

        return MCE_FPU_OK


    def addStep(self, first, last,
                asteps, apause, aclockwise,
                bsteps, bpause, bclockwise):

        verbose = self.opts.verbosity > 4

        ovflag = self.can_overflow
        if ovflag:                     # this might having been set
            self.can_overflow = False  # by a Unix signal handler
            if ovflag == 'HW':
                return MCE_ERR_CAN_OVERFLOW_HW, None
            else:
                return MCE_ERR_CAN_OVERFLOW_SW, None

        if self.state == FPST_LOCKED :
            return MCE_NOTIFY_COMMAND_IGNORED

        if self.state not in [ FPST_AT_DATUM,
                               FPST_LOADING,
                               FPST_READY_FORWARD,
                               FPST_READY_REVERSE,
                               FPST_RESTING]:

            return MCE_ERR_INVALID_COMMAND


        if self.running_wave:
            return MCE_ERR_INVALID_COMMAND, None

        if (self.nwave_entries == 0) and (not first):
            return WAVEFORM_REJECTED, WAVEFORM_SEQUENCE

        verbose = self.opts.verbosity > 9

        if first:
            self.nwave_entries = 0
            self.wave_ready = False
            self.wave_valid = False
            self.wave_reversed = False
            # clear abort status flag
            self.abort_wave = False
            self.state = FPST_LOADING

        n = self.nwave_entries

        if n  >= MAX_WAVE_ENTRIES:
            self.state = FPST_RESTING
            return WAVEFORM_REJECTED, WAVEFORM_TOO_BIG


        nwave_entries = self.nwave_entries


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
            self.state = FPST_READY_FORWARD
            if verbose:
                print("fpu #%i: wavetable ready, n=%i, content=%s" % (

                    self.fpu_id, n, (self.steps[0:n], self.pause[0:n], self.clockwise[0:n])))
            else:
                print("fpu #%i: wavetable ready (%i sections)" % (self.fpu_id, n))

        return MCE_FPU_OK, WAVEFORM_OK


    def alpha_switch_on(self, asteps=None):
        if asteps == None:
            asteps = self.alpha_steps
        alpha_steps = asteps + self.aoff_steps
        return ( ((alpha_steps > MIN_ALPHA_OFF)
                  and (alpha_steps < MIN_ALPHA_ON))
                 or ( ((alpha_steps > MAX_ALPHA_ON)
                  and (alpha_steps < MAX_ALPHA_OFF))))


    def move_alpha(self, new_alpha, limit_callback):

        if new_alpha == self.alpha_steps:
            return
        old_switch = self.alpha_switch_on(self.alpha_steps)

        new_switch = self.alpha_switch_on(new_alpha)

        out_direction = (new_alpha < MIN_ALPHA_ON) or (new_alpha > MAX_ALPHA_ON)
        alpha_offset = self.aoff_steps

        cur_direction = np.sign(new_alpha - self.alpha_steps)

        old_alpha = self.alpha_steps
        self.alpha_steps = new_alpha
        alpha_last_direction = self.alpha_switch_direction

        if new_switch:
            self.alpha_limit_breach = False

        if new_switch and (not old_switch):
            self.alpha_switch_direction = cur_direction
        # an alpha reach is defined as:
        # 1. we go off the switch
        # 2. we are moving
        # 3. direction is the same as during the last switch reak


        if ((not new_switch)
            and old_switch
            and (cur_direction != 0)
            and (cur_direction == alpha_last_direction)):
            self.alpha_limit_breach = True
            self.was_initialized = False
            self.wave_ready = False
            self.wave_valid = False
            print("LIMIT BREACH detected: last pos = %f, new pos =%f"
                  "alpha move: old switch = %r, new switch=%r"
                  "last_dirction = %i, cur_direction=%i"
                  % (old_alpha, new_alpha,
                     old_switch, new_switch,
                     alpha_last_direction, cur_direction))

            limit_callback(self)

            if (new_alpha + alpha_offset) < MIN_ALPHA_OFF:
                self.alpha_steps = MIN_ALPHA_OFF
            elif (new_alpha + alpha_offset) > MAX_ALPHA_OFF:
                self.alpha_steps = MAX_ALPHA_OFF

            raise LimitBreachException("An alpha limit breach occured")
        else:
            if (new_alpha + alpha_offset) < MIN_ALPHA_CRASH:
                raise CrashException("An min alpha crash occured")
            if (new_alpha + alpha_offset) > MAX_ALPHA_CRASH:
                print("new_alpha=",new_alpha,
                      ", alpha_soffset=", alpha_offset,
                      ", MAX_ALPHA_CRASH=", MAX_ALPHA_CRASH)
                raise CrashException("An max alpha crash occured")



    def move_beta(self, newbeta, collision_callback):
        beta_offset = self.boff_steps
        self.beta_steps = newbeta
        if ((newbeta + beta_offset) < MIN_BETA) and self.collision_protection_active :
            self.beta_steps = MIN_BETA
            self.is_collided = True
            self.was_initialized = False
            self.wave_ready = False
            self.wave_valid = False
            collision_callback(self)
            raise BetaCollisionException("a beta arm collision was detected (due to running into the lower stop)")
        elif ((newbeta + beta_offset) > MAX_BETA) and self.collision_protection_active:
            self.is_collided = True
            self.was_initialized = False
            self.beta_steps = MAX_BETA
            self.wave_ready = False
            self.wave_valid = False
            collision_callback(self)
            raise BetaCollisionException("a beta arm collision was detected (due to running into the upper stop)")
        elif self.is_collided :
            self.was_initialized = False
            self.wave_ready = False
            self.wave_valid = False
            collision_callback(self)
            raise BetaCollisionException("a beta arm collision was detected (due to signal)")
        else:
            if (newbeta + beta_offset) < MIN_BETA_CRASH:
                raise CrashException("An min beta crash occured")
            if (newbeta + beta_offset) > MAX_BETA_CRASH:
                raise CrashException("An max beta crash occured")

    def start_findDatum(self, auto_datum):
        if self.state == FPST_LOCKED:
            return MCE_NOTIFY_COMMAND_IGNORED

        if self.state in [FPST_ABORTED, FPST_OBSTACLE_ERROR, FPST_LOADING,
                          FPST_DATUM_SEARCH, FPST_MOVING]:
            return MCE_ERR_INVALID_COMMAND

        if self.is_collided:
            # only send an error message
            errcode = MCE_WARN_COLLISION_DETECTED
            self.state = FPST_OBSTACLE_ERROR
        elif self.alpha_limit_breach:
            errcode = MCE_WARN_LIMIT_SWITCH_BREACH
            self.state = FPST_OBSTACLE_ERROR
        elif auto_datum and (not self.was_initialized):
            errcode = MCE_ERR_AUTO_DATUM_UNINITIALIZED # not initialized, reject automatic datum search
            if self.state not in [FPST_LOCKED, FPST_AT_DATUM, FPST_ABORTED, FPST_OBSTACLE_ERROR]:
                self.state = FPST_UNINITIALIZED
        elif self.alpha_switch_on():
            errcode = MCE_ERR_DATUM_ON_LIMIT_SWITCH # alpha on limit switch, reject datum command
            if self.state not in [FPST_LOCKED, FPST_AT_DATUM, FPST_ABORTED, FPST_OBSTACLE_ERROR]:
                self.state = FPST_UNINITIALIZED
        else:
            # send confirmation and spawn findDatum method call
            errcode = MCE_FPU_OK
            self.was_initialized = False
            self.state = FPST_DATUM_SEARCH

        return errcode


    def findDatum(self, sleep, limit_callback, collision_callback,
                  skip_alpha=False, skip_beta=False,
                  auto_datum=False, anti_clockwise=False,
                  disable_timeout=False):


        printtime()
        wait_interval_sec = 0.1
        alpha_speed = -50.0 # steps per time interval
        beta_speed = -50.0 # steps per interval

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

        beta_offset = self.boff_steps
        alpha_offset = self.aoff_steps

        d_offset = self.opts.alpha_datum_offset
        alpha_real_deg =  (self.alpha_steps + alpha_offset) / StepsPerDegreeAlpha + d_offset
        beta_real_deg =  (self.beta_steps + beta_offset) / StepsPerDegreeBeta

        print("FPU #%i: findDatum is now at (%i, %i) steps = (%7.2f, %7.2f) degree\n" % (
            self.fpu_id, self.alpha_steps, self.beta_steps, alpha_real_deg, beta_real_deg))


        last_beta_stepnum = self.beta_steps + beta_offset
        last_alpha_stepnum = self.alpha_steps + alpha_offset

        beta_crossed_zero = False
        alpha_crossed_zero = False
        alpha_timeout_limit = self.opts.datum_alpha_timeout_steps
        beta_timeout_limit = self.opts.datum_beta_timeout_steps
        start_alpha_steps = self.alpha_steps
        start_beta_steps = self.beta_steps
        self.datum_timeout = False
        alpha_datumed = False
        beta_datumed = False
        use_timeout = self.opts.fw_version >= (1, 4, 0) and (not disable_timeout)
        print("FPU %i : timeouts enabled: %r" % (self.fpu_id, use_timeout))
        self.state = FPST_DATUM_SEARCH

        while True:
            # the model here is that crossing physical zero
            # edge-triggers the stop signal for both arms

            beta_stepnum  = self.beta_steps + beta_offset
            beta_zero_crossing= (last_beta_stepnum * beta_stepnum) <= 0
            beta_crossed_zero = beta_crossed_zero or beta_zero_crossing

            alpha_stepnum  = self.alpha_steps + alpha_offset
            alpha_zero_crossing= (last_alpha_stepnum * alpha_stepnum) <= 0
            alpha_crossed_zero = alpha_crossed_zero or alpha_zero_crossing


            alpha_ready = alpha_crossed_zero or skip_alpha
            beta_ready = beta_crossed_zero or skip_beta


            if alpha_ready and beta_ready:
                alpha_datumed = alpha_ready and (not skip_alpha)
                beta_datumed = beta_ready and (not skip_beta)
                break

            sleep(wait_interval_sec)

            if self.abort_wave:
                print("ABORTING DATUM SEARCH FOR FPU", self.fpu_id);
                self.at_datum = False
                self.state = FPST_ABORTED
                break



            try:
                if not alpha_ready:
                    new_alpha = self.alpha_steps + int(alpha_speed)
                    self.move_alpha(new_alpha, limit_callback)
                else:
                    new_alpha = self.alpha_steps
            except LimitBreachException:
                print("Alpha limit breach for FPU  %i" % self.fpu_id)
                self.abort_wave = True
                self.state = FPST_OBSTACLE_ERROR
                break

            try:
                if self.is_collided:
                    print("Beta collision for FPU  %i still active" % self.fpu_id)
                    self.abort_wave = True
                    self.is_collided = True
                    self.state = FPST_OBSTACLE_ERROR
                    break

                if not beta_ready:
                    # this is much simplified: search until crossing the zero point
                    if not beta_crossed_zero:
                        new_beta = self.beta_steps + int(beta_speed * beta_sign)
                        self.move_beta(new_beta, collision_callback)
                    else:
                        new_beta = self.beta_steps
                else:
                    new_beta = self.beta_steps

            except BetaCollisionException:
                print("Beta collision for FPU  %i" % self.fpu_id)
                self.abort_wave = True
                self.is_collided = True
                self.state = FPST_OBSTACLE_ERROR
                break

            if use_timeout and (abs(new_alpha - start_alpha_steps) >= alpha_timeout_limit):
                print("FPU %i: step number exceeds alpha time-out step count of %i, aborting" % (self.fpu_id, alpha_timeout_limit))
                self.abort_wave = True
                self.datum_timeout = True
                alpha_datumed = alpha_ready and (not skip_alpha)
                beta_datumed = beta_ready and (not skip_beta)
                break

            if use_timeout and (abs(new_beta - start_beta_steps) >= beta_timeout_limit):
                print("FPU %i: step number exceeds beta time-out step count of %i, aborting" % (self.fpu_id, beta_timeout_limit))
                self.abort_wave = True
                self.datum_timeout = True
                alpha_datumed = alpha_ready and (not skip_alpha)
                beta_datumed = beta_ready and (not skip_beta)
                break


            alpha_real_deg =  (self.alpha_steps + alpha_offset) / StepsPerDegreeAlpha + d_offset
            beta_real_deg =  (self.beta_steps + beta_offset) / StepsPerDegreeBeta

            print("FPU #%i: findDatum is now at (%i, %i) steps = (%7.2f, %7.2f) degree" % (
                self.fpu_id, self.alpha_steps, self.beta_steps, alpha_real_deg, beta_real_deg))


        if alpha_datumed:
            self.alpha_steps = 0
            self.aoff_steps = 0
            if skip_beta:
                print("FPU #%i: alpha datum reached" % self.fpu_id)

        if beta_datumed:
            self.beta_steps = 0
            self.boff_steps = 0
            if skip_alpha:
                print("FPU #%i: beta datum reached" % self.fpu_id)

        if alpha_datumed and beta_datumed:
            print("FPU #%i: alpha and beta datum reached" % self.fpu_id)
            self.state = FPST_AT_DATUM
            self.was_initialized = True
        else:
            if self.state not in [FPST_OBSTACLE_ERROR, FPST_ABORTED]:
                self.state = FPST_UNINITIALIZED

        if self.abort_wave:
            if self.state != FPST_OBSTACLE_ERROR:
                self.state = FPST_ABORTED
        else:
            if not (skip_alpha or skip_beta):
                self.at_datum = True
                self.was_initialized = True
                print("FPU #%i: datum found" % self.fpu_id)
            else:
                print("FPU #%i: partial datum operation finished" % self.fpu_id)

        alpha_real_deg =  (self.alpha_steps + self.aoff_steps) / StepsPerDegreeAlpha + d_offset
        beta_real_deg =  (self.beta_steps + self.boff_steps) / StepsPerDegreeBeta

        print("FPU #%i: findDatum stopped at (%i, %i) steps = (%7.2f, %7.2f) degree" % (
            self.fpu_id, self.alpha_steps, self.beta_steps, alpha_real_deg, beta_real_deg))

        printtime()

        if self.is_collided:
                # only send an error message
                errcode = MCE_WARN_COLLISION_DETECTED
        elif self.alpha_limit_breach:
            errcode = MCE_WARN_LIMIT_SWITCH_BREACH
        elif self.datum_timeout:
            errcode = MCE_ERR_DATUM_TIME_OUT
        elif self.alpha_switch_on():
            errcode = MCE_ERR_DATUM_ON_LIMIT_SWITCH # alpha on limit switch, reject datum command
        else:
            if skip_alpha and (not skip_beta):
                errcode = MCE_NOTIFY_DATUM_ALPHA_ONLY
            elif skip_beta and (not skip_alpha):
                errcode = MCE_NOTIFY_DATUM_BETA_ONLY
            else:
                errcode = MCE_FPU_OK

        return errcode



    def abortMotion(self, sleep):
        if self.state == FPST_LOCKED:
            return MCE_NOTIFY_COMMAND_IGNORED

        if self.state in [ FPST_DATUM_SEARCH, FPST_MOVING ]:
            self.abort_wave = True
            self.was_initialized = False
            self.wave_ready = False
            self.wave_valid = False
            self.state = FPST_ABORTED
            errcode = MCE_FPU_OK
        elif self.state in [ FPST_LOADING, READY_FORWARD, READY_REVERSE ]:
            self.wave_ready = False
            self.wave_valid = False
            errcode = MCE_FPU_OK
            self.state = FPST_RESTING
        elif self.state == FPST_RESTING:
            self.wave_ready = False
            self.wave_valid = False
            errcode = MCE_FPU_OK
        else:
            errcode = MCE_NOTIFY_COMMAND_IGNORED

        return errcode


    def freeBetaCollision(self, direction):
        if self.state == FPST_LOCKED:
            return MCE_NOTIFY_COMMAND_IGNORED

        if not ( (direction == 0) or (direction == 1)):
            return MCE_ERR_INVALID_PARAMETER

        if not self.state == FPST_OBSTACLE_ERROR:
            return MCE_ERR_INVALID_COMMAND

        UNTANGLE_STEPS = 10
        alpha_offset = self.aoff_steps
        beta_offset = self.boff_steps

        self.collision_protection_active = False

        old_beta_steps = self.beta_steps

        if direction == REQD_CLOCKWISE:
            self.beta_steps -= UNTANGLE_STEPS
        else:
            self.beta_steps += UNTANGLE_STEPS

        d_offset = self.opts.alpha_datum_offset
        alpha_real_deg =  (self.alpha_steps + alpha_offset) / StepsPerDegreeAlpha + d_offset
        beta_real_deg =  (self.beta_steps + beta_offset) / StepsPerDegreeBeta

        print("freeBetaCollsion: moving FPU # %i from (%i,%i) to (%i, %i) = real (%5.2f, %5.2f) deg" % (
            self.fpu_id, self.alpha_steps, old_beta_steps,
            self.alpha_steps, self.beta_steps,
            alpha_real_deg, beta_real_deg))

        if ((self.beta_steps + beta_offset) >= MAX_BETA) or (
                (self.beta_steps + beta_offset) <= MIN_BETA):
            self.is_collided = True
            print("FPU #%i: collision ongoing" % self.fpu_id)
        else:
            self.is_collided = False
            print("FPU #%i: collision resolved" % self.fpu_id)

        return MCE_FPU_OK

    def enableBetaCollisionProtection(self):
        self.is_collided = False
        self.is_initialized = False
        self.wave_ready = False
        self.wave_valid = False
        self.collision_protection_active = True
        self.state = FPST_RESTING
        return MCE_FPU_OK


    def freeAlphaLimitBreach(self, direction):
        if self.state == FPST_LOCKED:
            return MCE_NOTIFY_COMMAND_IGNORED

        if not ( (direction == 0) or (direction == 1)):
            return MCE_ERR_INVALID_PARAMETER

        if not self.state == FPST_OBSTACLE_ERROR:
            return MCE_ERR_INVALID_COMMAND

        UNTANGLE_STEPS = 10
        alpha_offset = self.aoff_steps
        beta_offset = self.boff_steps

        self.collision_protection_active = False

        old_alpha_steps = self.alpha_steps

        if direction == REQD_CLOCKWISE:
            self.alpha_steps -= UNTANGLE_STEPS
        else:
            self.alpha_steps += UNTANGLE_STEPS

        d_offset = self.opts.alpha_datum_offset

        alpha_real_deg =  (self.alpha_steps + alpha_offset) / StepsPerDegreeAlpha + d_offset
        beta_real_deg =  (self.beta_steps + beta_offset) / StepsPerDegreeBeta

        print("freeAlphaLimitBreach: moving FPU # %i from (%i,%i) to (%i, %i) = real (%5.2f, %5.2f) deg" % (
            self.fpu_id, old_alpha_steps, self.beta_steps,
            self.alpha_steps, self.beta_steps,
            alpha_real_deg, beta_real_deg))

        if self.alpha_switch_on(self.alpha_steps):
            self.alpha_limit_breach = False
            print("FPU #%i: limit breach resolved" % self.fpu_id)
        else:
            if ((self.alpha_steps + alpha_offset) >= MAX_ALPHA_OFF) or (
                    (self.alpha_steps + alpha_offset) <= MIN_ALPHA_OFF):
                self.alpha_limit_breach = True
            print("FPU #%i: limit breach ongoing" % self.fpu_id)

        return MCE_FPU_OK

    def enableAlphaLimitBreachProtection(self):
        self.alpha_limit_breach = False
        self.collision_protection_active = True
        self.is_initialized = False
        self.wave_ready = False
        self.wave_valid = False
        self.state = FPST_RESTING
        return MCE_FPU_OK


    def start_executeMotion(self):
        if self.state == FPST_LOCKED:
            return MCE_NOTIFY_COMMAND_IGNORED

        if self.is_collided:
            # collision active, only send an error message
            errcode = MCE_WARN_COLLISION_DETECTED
            self.state = FPST_OBSTACLE_ERROR
        elif self.alpha_limit_breach:
            errcode = MCE_WARN_LIMIT_SWITCH_BREACH
            self.state = FPST_OBSTACLE_ERROR
        elif not self.wave_ready:
            # wavetable is not ready
            print("FPU #", self.fpu_id, ": wave table is not ready, sending response code", MCE_ERR_WAVEFORM_NOT_READY)
            errcode =  MCE_ERR_WAVEFORM_NOT_READY
        elif not self.wave_valid:
            # wavetable is not valid
            print("FPU #", self.fpu_id, ": wave table is not valid, sending response code", MCE_WAVEFORM_BADVALUE)
            errcode =  MCE_WAVEFORM_BADVALUE
        elif self.running_wave:
            # FPU already moving
            errcode = MCE_ERR_INVALID_COMMAND
        elif self.state not in [FPST_READY_FORWARD, FPST_READY_REVERSE]:
            errcode = MCE_ERR_INVALID_COMMAND
        else:
            self.state = FPST_MOVING
            errcode = MCE_FPU_OK

        return errcode


    def executeMotion(self, sleep, limit_callback, collision_callback):
        printtime()
        if self.running_wave :
            raise RuntimeError("FPU is already moving")

        if not (self.wave_ready):
            raise RuntimeError("wavetable not ready")


        beta_offset = self.boff_steps

        self.running_wave = True
        self.step_timing_fault = False

        if self.wave_reversed:
            wt_sign = -1
        else:
            wt_sign = 1

        #simulate between 0 and 10 ms of latency
        latency_secs = random.uniform(0, 10) / 1000.
        sleep(latency_secs)

        alpha_limit_breach = False
        section = -1
        for k in range(self.nwave_entries):
            section = k
            if self.abort_wave:
                # flag was set from abortMotion command
                self.state = FPST_ABORTED
                break

            if not self.wave_reversed:
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
            d_offset = self.opts.alpha_datum_offset # conventional angle at datum position
            alpha_real_deg =  (new_alpha + alpha_offset) / StepsPerDegreeAlpha + d_offset
            beta_real_deg =  (new_beta + beta_offset) / StepsPerDegreeBeta

            if self.opts.verbosity > 0:
                alpha_swon = self.alpha_switch_on(new_alpha)
                print("section %i: moving FPU %i by (%i,%i) to (%i, %i) = real (%5.2f, %5.2f) deg, alpha_switch_on=%r" % (
                    n, self.fpu_id, delta_alpha, delta_beta, new_alpha, new_beta,
                    alpha_real_deg, beta_real_deg, alpha_swon))

            frame_time = 0.25
            sleep(frame_time)
            if self.abort_wave:
                self.state = FPST_ABORTED
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
                alpha_limit_breach = True
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
        elif alpha_limit_breach:
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

        if self.is_collided:
            # only send an error message
            errcode = MCE_WARN_COLLISION_DETECTED
            self.state = FPST_OBSTACLE_ERROR
            self.was_initialized = False
        elif self.alpha_limit_breach:
            errcode = MCE_WARN_LIMIT_SWITCH_BREACH
            self.state = FPST_OBSTACLE_ERROR
            self.was_initialized = False
        elif self.step_timing_fault:
            # send error message
            errcode = MCE_WARN_STEP_TIMING_ERROR
            self.state = FPST_ABORTED
        elif self.abort_wave:
            # send error message
            errcode = MCE_WARN_STEP_TIMING_ERROR
            self.state = FPST_ABORTED
            self.was_initialized = False
        else:
            # in version 1, other cases do not have
            # status flag information
            errcode = MCE_FPU_OK
            self.state = FPST_RESTING

        return errcode

    def lockUnit(self):
        if self.state == FPST_LOCKED:
            return MCE_NOTIFY_COMMAND_IGNORED

        # don't allow locking when moving
        if self.state in [ FPST_MOVING, FPST_DATUM_SEARCH]:
            return MCE_ERR_INVALID_COMMAND

        if self.state == FPST_ABORTED:
            self.was_initialized = False
            self.wave_valid = False
            self.wave_ready = False

        self.state = FPST_LOCKED

        return MCE_FPU_OK

    def unlockUnit(self):

        if self.state != FPST_LOCKED:
            return MCE_NOTIFY_COMMAND_IGNORED

        # we try to restore the pervious state -
        # modulo any collisions which have occured since!

        if self.is_collided:
            self.state = FPST_OBSTACLE_ERROR
            self.was_initialized = False
        elif self.alpha_limit_breach:
            self.state = FPST_OBSTACLE_ERROR
            self.was_initialized = False
        elif self.was_initialized :
            if (self.alpha_steps == 0) and (self.beta_steps == 0):
                self.state = FPST_AT_DATUM
            elif (self.wave_ready and self.wave_valid):
                if self.wave_reversed:
                    self.state = FPST_READY_REVERSE
                else:
                    self.state = FPST_READY_FORWARD
            else:
                self.state = FPST_RESTING
        else:
            self.state = FPST_UNINITIALIZED

        return MCE_FPU_OK

    def enableMove(self):

        # don't allow command when moving
        if self.state in [ FPST_MOVING, FPST_DATUM_SEARCH]:
            return MCE_ERR_INVALID_COMMAND

        if self.state not in [FPST_ABORTED, FPST_UNINITIALIZED, FPST_RESTING,
                              FPST_READY_FORWARD, FPST_READY_REVERSE]:
            # (if locked, needs unlocking first)
            return MCE_NOTIFY_COMMAND_IGNORED
        print("FPU # %i: setting state to FPST_RESTING" % self.fpu_id)
        self.state = FPST_RESTING

        return MCE_FPU_OK

    def checkIntegrity(self):
        if self.state in [ FPST_MOVING, FPST_DATUM_SEARCH,
                           FPST_READY_FORWARD, FPST_READY_REVERSED]:
            return MCE_ERR_INVALID_COMMAND
        # returns CRC32 checksum of 'firmware' (here, the class source code)
        source_string = "".join(inspect.getsourcelines(FPU)[0])
        crc32val= binascii.crc32(source_string) & 0xffffffff
        print("CRC32 for FPU # %i = %s" % (self.fpu_id, hex(crc32val)))
        # ok
        return crc32val, MCE_FPU_OK




class SignalHandler(object):
    """Context manager for handling a signal
    while waiting for command completion.
    """

    def __init__(self, sig=signal.SIGINT):
        self.sig = sig

    def __enter__(self):

        self.interrupted = False
        self.released = False

        self.original_handler = signal.getsignal(self.sig)

        def handler(signum, frame):
            self.release()
            self.interrupted = True


        return self

def reset_handler(signum, frame):
    print("resetting FPUs")
    for fpu_id, fpu in enumerate(FPUGrid):
        fpu.resetFPU(fpu_id, time.sleep)

def collision_handler(signum, frame):
    print("generating a collision")
    for fpu_id, fpu in enumerate(FPUGrid):
        fpu.is_collided = True


def overflow_handler(signum, frame):
    print("generating a CAN overflow")
    for fpu_id, fpu in enumerate(FPUGrid):
        fpu.can_overflow = True

def colltest_handler(signum, frame):
    try:
        fpu_id = int(open("/var/tmp/colltest.fpuid").readline())
        print("setting collision test flag for FPU # %i" % fpu_id)
        FPUGrid[fpu_id].coll_test = True
    except:
        pass

def init_FPUGrid(options, num_fpus):
    FPUGrid[:] = [FPU(i, options) for i in range(num_fpus) ]
    signal.signal(signal.SIGHUP, reset_handler)
    signal.signal(signal.SIGUSR1, collision_handler)
    signal.signal(signal.SIGUSR2, overflow_handler)
    signal.signal(signal.SIGRTMIN, colltest_handler)
