// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-05-04  TODO: Put comment here
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME FPUConstants.h
//
// TODO: Put description here
//
////////////////////////////////////////////////////////////////////////////////

#ifndef FPUCONSTANTS_H
#define FPUCONSTANTS_H


// ********** TODO: This file is being gradually adapted from fpu_constants.py


namespace mpifps
{

const double MAX_ACCELERATION_FACTOR = 1.4;     // This constant factor is obsolete for
                                                // newer waveform rulesets. For details,
                                                // see driver manual sections 6.5.8, 6.9.2
                                                // and 17.1.

// This should match the values in include/EtherCANInterfaceConfig.h
const double MOTOR_MIN_STEP_FREQUENCY = 500.0;  // Minimum stepper motor frequency
const double MOTOR_MAX_STEP_FREQUENCY = 2000.0; // maximum stepper motor frequency
const double MOTOR_MAX_START_FREQUENCY = 550.0; // Minimum start frequency
const int MAX_STEP_DIFFERENCE = 50;             // Maximum change in steps per segment between segments

// Used for waveform generation, can be changed for testing motor limits
const int MOTOR_MAX_ACCELERATION = MAX_STEP_DIFFERENCE;
const int MOTOR_MAX_DECELERATION = MAX_STEP_DIFFERENCE;

}

#endif  // FPUCONSTANTS_H