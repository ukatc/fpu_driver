#!/usr/bin/python

AlphaGearRatio 	= 2050.175633 # actual gear ratio
BetaGearRatio 	= 1517.662482 # actual gear ratio


# There are 20 steps per revolution on the non-geared side, so:
StepsPerRevolution = 20.0
DegreePerRevolution = 360.0

# Note that these numbers must not be confounded with actual calibrated values!

StepsPerDegreeAlpha = float(StepsPerRevolution * AlphaGearRatio) / DegreePerRevolution
StepsPerDegreeBeta = float(StepsPerRevolution * BetaGearRatio) / DegreePerRevolution

ALPHA_MIN_DEGREE = -180
ALPHA_MAX_DEGREE = +179.6
BETA_MIN_DEGREE = -179.3
BETA_MAX_DEGREE = 150.3

ALPHA_DATUM_OFFSET=-180.0
BETA_DATUM_OFFSET=0.0

DEFAULT_FREE_BETA_RETRIES=3
FREE_BETA_STEPCOUNT=10

#MIN_ALPHA = ALPHA_MIN_DEGREE * StepsPerDegreeAlpha
#MAX_ALPHA = ALPHA_MAX_DEGREE * StepsPerDegreeAlpha

#MIN_BETA = BETA_MIN_DEGREE * StepsPerDegreeBeta 
#MAX_BETA = BETA_MAX_DEGREE * StepsPerDegreeBeta 
