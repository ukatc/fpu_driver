// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-05-04  Created (translated from Python fpu_constants.py).
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME FPUConstants.h
//
// FPU constants for sharing between files.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef FPUCONSTANTS_H
#define FPUCONSTANTS_H

// TODO: This file is being gradually adapted from fpu_constants.py

// TODO: Eventually change the naming conventions to
// lowercase_with_underscores - have just been keeping the uppercase + camel
// case syntax so that same as original Python for now

namespace mpifps
{

const double AlphaGearRatio = 2050.175633;   // Actual gear ratio
const double BetaGearRatio = 1517.662482;    // actual gear ratio

// There are 20 steps per revolution on the non-geared side, so:
const double StepsPerRevolution = 20.0;
const double DegreePerRevolution = 360.0;

// Note that these numbers must not be confused with actual calibrated values!
// TODO: The Python versions in fpu_constants.py have "float()" functions in
// them - what is the purpose of this, and is the following C++ version OK?
const double StepsPerDegreeAlpha =
                (StepsPerRevolution * AlphaGearRatio) / DegreePerRevolution;
const double StepsPerDegreeBeta =
                (StepsPerRevolution * BetaGearRatio) / DegreePerRevolution;

const double BETA_MIN_HWPROT_DEGREE = -179.4;
const double BETA_MAX_HWPROT_DEGREE = 150.4;

const double ALPHA_MIN_HARDSTOP_DEGREE = -183.2;
const double ALPHA_MAX_HARDSTOP_DEGREE = 181.8;

const int DEFAULT_FREE_BETA_RETRIES = 6;
const int FREE_BETA_STEPCOUNT = 10;

const int DEFAULT_FREE_ALPHA_RETRIES = 6;
const int FREE_ALPHA_STEPCOUNT = 11;

// This constant factor is obsolete for newer waveform rulesets. For details,
// see driver manual sections 6.5.8, 6.9.2 and 17.1.
 const double MAX_ACCELERATION_FACTOR = 1.4;

// This should match the values in include/EtherCANInterfaceConfig.h
// Minimum stepper motor frequency
const double MOTOR_MIN_STEP_FREQUENCY = 500.0;
// maximum stepper motor frequency
const double MOTOR_MAX_STEP_FREQUENCY = 2000.0;
// Minimum start frequency
const double MOTOR_MAX_START_FREQUENCY = 550.0;
// Maximum change in steps per segment between segments
const int MAX_STEP_DIFFERENCE = 50;

// Used for waveform generation, can be changed for testing motor limits
const int MOTOR_MAX_ACCELERATION = MAX_STEP_DIFFERENCE;
const int MOTOR_MAX_DECELERATION = MAX_STEP_DIFFERENCE;

// Overflow / underflow representations in binary FPU step counters - these are
// intentionally asymmetric for the alpha arm counter
const int ALPHA_UNDERFLOW_COUNT = -10000;
const int ALPHA_OVERFLOW_COUNT = ALPHA_UNDERFLOW_COUNT + (1 << 16) - 1;

const int BETA_UNDERFLOW_COUNT = -0x8000;
const int BETA_OVERFLOW_COUNT = BETA_UNDERFLOW_COUNT + (1 << 16) - 1;

}

#endif  // FPUCONSTANTS_H