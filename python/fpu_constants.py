#!/usr/bin/python

from math import pi
from numpy import rad2deg, deg2rad

RADIAN_TO_DEGREE = 180.0 / pi # conversion factor


AlphaGearRatio 	= 2050.175633 # actual gear ratio
BetaGearRatio 	= 1517.662482 # actual gear ratio


# There are 20 steps per revolution on the non-geared side, so:
StepsPerRevolution = 20.0
DegreePerRevolution = 360.0

# Note that these numbers must not be confounded with actual calibrated values!

StepsPerDegreeAlpha = float(StepsPerRevolution * AlphaGearRatio) / DegreePerRevolution
StepsPerDegreeBeta = float(StepsPerRevolution * BetaGearRatio) / DegreePerRevolution

StepsPerRadianAlpha = StepsPerDegreeAlpha / deg2rad(1.0)
StepsPerRadianBeta = StepsPerDegreeBeta / deg2rad(1.0)

ALPHA_MIN_DEGREE = -180.2
ALPHA_MAX_DEGREE = +159.0
BETA_MIN_DEGREE = -179.3
BETA_MAX_DEGREE = 141.0

BETA_MIN_HWPROT_DEGREE = -179.4
BETA_MAX_HWPROT_DEGREE = 150.4

ALPHA_MIN_HARDSTOP_DEGREE = -183.2
ALPHA_MAX_HARDSTOP_DEGREE = +181.8

ALPHA_DATUM_OFFSET = -180.0
BETA_DATUM_OFFSET = 6.5

ALPHA_DATUM_OFFSET_RAD = deg2rad(ALPHA_DATUM_OFFSET)
BETA_DATUM_OFFSET_RAD = deg2rad(BETA_DATUM_OFFSET)

# NOTE: Constants DEFAULT_FREE_ALPHA_RETRIES and DEFAULT_FREE_BETA_RETRIES
# were not originally used anywhere within the Python code.
DEFAULT_FREE_BETA_RETRIES=6
FREE_BETA_STEPCOUNT=10

DEFAULT_FREE_ALPHA_RETRIES=6
FREE_ALPHA_STEPCOUNT=11

MAX_ACCELERATION_FACTOR=1.4 # this constant factor is obsolete for
                            # newer waveform rulesets. For details,
                            # see driver manual sections 6.5.8, 6.9.2
                            # and 17.1.

# this should match the values in include/EtherCANInterfaceConfig.h
MOTOR_MIN_STEP_FREQUENCY=500  # minimum stepper motor frequency
MOTOR_MAX_STEP_FREQUENCY=2000 # maximum stepper motor frequency
MOTOR_MAX_START_FREQUENCY=550 # minimum start frequency
MAX_STEP_DIFFERENCE=50        # maximum change in steps per segment between segments

# used for waveform generation, can be
# changed for testing motor limits
MOTOR_MAX_ACCELERATION=MAX_STEP_DIFFERENCE
MOTOR_MAX_DECELERATION=MAX_STEP_DIFFERENCE

MAXNUM_WAVEFORM_SEGMENTS = 256
WAVEFORM_SEGMENT_LENGTH_MS=125

# overflow / underflow representations in binary FPU step counters
# this is intentionally asymmetric for the alpha arm counter
ALPHA_UNDERFLOW_COUNT = - 10000
ALPHA_OVERFLOW_COUNT = ALPHA_UNDERFLOW_COUNT + (1 << 16) -1

BETA_UNDERFLOW_COUNT = - 0x8000
BETA_OVERFLOW_COUNT = BETA_UNDERFLOW_COUNT + (1 << 16) -1

if __name__ == '__main__':
    # List some of the main constants when this file is run from the
    # command line.
    print("Default parameters defined by this module.")
    print("------------------------------------------")

    print("Alpha limits: min-hardstop=%.2f, min=%.2f, max=%.2f, max-hardstop=%.2f (deg)." % \
        (ALPHA_MIN_HARDSTOP_DEGREE, ALPHA_MIN_DEGREE, ALPHA_MAX_DEGREE, ALPHA_MAX_HARDSTOP_DEGREE))
    print("Beta  limits: min-hardprot=%.2f, min=%.2f, max=%.2f, max-hardprot=%.2f (deg)." % \
        (BETA_MIN_HWPROT_DEGREE, BETA_MIN_DEGREE, BETA_MAX_DEGREE, BETA_MAX_HWPROT_DEGREE))
    print(" ")

    print("Alpha motor scale is %f steps per deg and %f steps per radian." % \
        (StepsPerDegreeAlpha, StepsPerRadianAlpha))
    print("Beta  motor scale is %f steps per deg and %f steps per radian." % \
        (StepsPerDegreeBeta, StepsPerRadianBeta))
    print(" ")

    freeAlphaDistance = float(FREE_ALPHA_STEPCOUNT) / StepsPerDegreeAlpha
    freeBetaDistance = float(FREE_BETA_STEPCOUNT) / StepsPerDegreeBeta
    print("freeAlphaLimitBreach moves %d steps (%f deg) and can be repeated %d times" \
          " covering a distance of %f deg." % \
          (FREE_ALPHA_STEPCOUNT, freeAlphaDistance, DEFAULT_FREE_ALPHA_RETRIES,
           DEFAULT_FREE_ALPHA_RETRIES*freeAlphaDistance))
    print("freeBetaCollision    moves %d steps (%f deg) and can be repeated %d times" \
          " covering a distance of %f deg." % \
          (FREE_BETA_STEPCOUNT, freeBetaDistance, DEFAULT_FREE_BETA_RETRIES,
           DEFAULT_FREE_BETA_RETRIES*freeBetaDistance))
    print(" ")

    print("Motor frequencies: min=%.1f, startstop=%.1f, max=%.1f (Hz)" % \
       (MOTOR_MIN_STEP_FREQUENCY, MOTOR_MAX_START_FREQUENCY, MOTOR_MAX_STEP_FREQUENCY))
    print("Max acceleration %.1f steps/s per %.1f ms segment." % \
       (MAX_STEP_DIFFERENCE, WAVEFORM_SEGMENT_LENGTH_MS))
    print(" ")

    print("Alpha underflow count=%d, overflow count=%d" % \
       (ALPHA_UNDERFLOW_COUNT, ALPHA_OVERFLOW_COUNT))
    print("Beta  underflow count=%d, overflow count=%d" % \
       (BETA_UNDERFLOW_COUNT, BETA_OVERFLOW_COUNT))
