// -*- mode: c++ -*-

///////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 UKRI. See file "LICENSE" for license information.
//
// ESO - VLT Project
//
// Copyright 2017 E.S.O,
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client
//                       sample
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME InterfaceState.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef INTERFACE_STATE_H
#define INTERFACE_STATE_H

namespace mpifps
{

enum E_InterfaceState
{
    // The driver can have the following non-operative states

    // Not yet initialized, or resource allocation failed, for example because
    // out-of-memory
    DS_UNINITIALIZED = 1,
                           
    // The driver is not connected, this is the state before connecting to the
    // gateway or after the TCP connection was lost. The latter would happen if
    // there is a serious extended failure, like a broken cable or a system
    // error with the gateway service.
    DS_UNCONNECTED = 2,

    // Driver is connected to gateways and working
    DS_CONNECTED = 3,

    // A fatal error occured, such as out-of-memory during ppoll()
    DS_ASSERTION_FAILED = 4,
};

// this is a one-bit parameter to several commands
enum E_REQUEST_DIRECTION
{
    REQD_ANTI_CLOCKWISE = 0,
    REQD_CLOCKWISE      = 1,
};

enum E_DATUM_SELECTION
{
    DASEL_BOTH  = 0,
    DASEL_ALPHA = 1,
    DASEL_BETA  = 2,
    DASEL_NONE  = 3,
};

enum E_DATUM_SEARCH_DIRECTION
{
    SEARCH_CLOCKWISE      = 0,
    SEARCH_ANTI_CLOCKWISE = 1,
    SEARCH_AUTO           = 2,
    SKIP_FPU              = 3,
};


}
#endif
