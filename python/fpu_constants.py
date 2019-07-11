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
BETA_MAX_DEGREE = 140.0

BETA_MIN_HWPROT_DEGREE = -179.4
BETA_MAX_HWPROT_DEGREE = 150.4

ALPHA_MIN_HARDSTOP_DEGREE = -183.2
ALPHA_MAX_HARDSTOP_DEGREE = +181.8

ALPHA_DATUM_OFFSET=-180.0
BETA_DATUM_OFFSET=0.0

ALPHA_DATUM_OFFSET_RAD= deg2rad(ALPHA_DATUM_OFFSET)
BETA_DATUM_OFFSET_RAD= deg2rad(BETA_DATUM_OFFSET)

DEFAULT_FREE_BETA_RETRIES=3
FREE_BETA_STEPCOUNT=10

MAX_ACCELERATION_FACTOR=1.4
MOTOR_MIN_STEP_FREQUENCY=500
MOTOR_MAX_STEP_FREQUENCY=2000
MOTOR_MAX_START_FREQUENCY=550
MOTOR_MAX_ACCELERATION=35
MOTOR_MAX_DECELERATION=35

MAXNUM_WAVEFORM_SEGMENTS = 128
WAVEFORM_SEGMENT_LENGTH_MS=250

# overflow / underflow representations in step counter
ALPHA_UNDERFLOW_COUNT = - 10000
ALPHA_OVERFLOW_COUNT = ALPHA_UNDERFLOW_COUNT + (1 << 16) -1

BETA_UNDERFLOW_COUNT = - 0x8000
BETA_OVERFLOW_COUNT = BETA_UNDERFLOW_COUNT + (1 << 16) -1
