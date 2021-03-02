// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client sample
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME InterfaceConstants.h
//
// This file defines global constants which reflect some
// properties of the hardware.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef INTERFACECONSTANTS_H
#define INTERFACECONSTANTS_H

#include "ethercan/CAN_Constants.h"

#ifdef DEBUG
#include <stdio.h>
#endif

namespace mpifps
{

// Define default value for more convenient testing
static const char DEFAULT_GATEWAY_IP[]= "192.168.0.10";
static const int DEFAULT_GATEWAY_PORT = 4700;

const double SOCKET_TIMEOUT_SECS = 20.0;

// Number of gateways for the positioner grid
const int MAX_NUM_GATEWAYS = 3;

// Maximum number of fibre positioner units
const int MAX_NUM_POSITIONERS = (MAX_NUM_GATEWAYS
                                 * ethercanif::BUSES_PER_GATEWAY
                                 * ethercanif::FPUS_PER_BUS);

const int MAX_FPUS_PER_GATEWAY = (ethercanif::BUSES_PER_GATEWAY
                                  * ethercanif::FPUS_PER_BUS);

const bool USE_REALTIME_SCHEDULING = false;

// Design scaling of stepper motors.
// From David Atkinson: These are the actual gear ratios that I use in my IDL
// positional repeatability code. I was given these numbers by David
// Montgomery:
const double ALPHA_GEAR_RATIO = 2050.175633; // Actual gear ratio
const double BETA_GEAR_RATIO = 1517.662482; // Actual gear ratio

// There are 20 steps per revolution on the non-geared side, so:
const double STEPS_PER_REVOLUTION = 20.0;
const double DEGREE_PER_REVOLUTION = 360.0;

// Note that these numbers must not be confused with actual calibrated values!
const double STEPS_PER_DEGREE_ALPHA =
            (STEPS_PER_REVOLUTION * ALPHA_GEAR_RATIO) / DEGREE_PER_REVOLUTION;
const double STEPS_PER_DEGREE_BETA =
            (STEPS_PER_REVOLUTION * BETA_GEAR_RATIO) / DEGREE_PER_REVOLUTION;

const double ALPHA_DATUM_OFFSET = -180.0;
const double BETA_DATUM_OFFSET = 0.0;

// Duration of one segment of a waveform
const double WAVEFORM_SEGMENT_DURATION_MS = 125;

const int DEFAULT_WAVEFORM_RULESET_VERSION = 5;
}
#endif
