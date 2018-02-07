// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
// ESO - VLT Project
//
// Copyright 2017 E.S.O,
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// Pablo Gutierrez 2017-07-22  created CAN client sample
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client sample
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME DriverConstants.h
//
// This file defines global constants which reflect some
// properties of the hardware.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef DRIVER_CONSTANTS_H
#define DRIVER_CONSTANTS_H

#include "canlayer/CAN_Constants.h"

#ifdef DEBUG
#include <stdio.h>
#endif

namespace mpifps
{


// define default value for more convenient testing.
static const char DEFAULT_GATEWAY_IP[]= "192.168.0.10";
static const int DEFAULT_GATEWAY_PORT = 4700;


// number of gateways for the positioner grid
const int MAX_NUM_GATEWAYS = 3;

// maximum number of fibre positioner units
const int MAX_NUM_POSITIONERS = (MAX_NUM_GATEWAYS
                                 * canlayer::BUSES_PER_GATEWAY
                                 * canlayer::FPUS_PER_BUS);

const bool USE_REALTIME_SCHEDULING = false;

}
#endif
