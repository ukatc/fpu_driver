// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 UKRI. See file "LICENSE" for license information.
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client sample
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME I_ResponseHandler.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#ifndef I_RESPONSE_HANDLER_H
#define I_RESPONSE_HANDLER_H

#include <stdint.h>

#include "CAN_Constants.h"
namespace mpifps
{

namespace ethercanif
{

class I_ResponseHandler
{
public:
    I_ResponseHandler() {};

    virtual ~I_ResponseHandler() {};

    // method to handle any incoming CAN response message.
    // the implementation will set the appropiate status information
    // for the corresponsing FPUs
    virtual void handleFrame(int const gateway_id, uint8_t const command_buffer[MAX_UNENCODED_GATEWAY_MESSAGE_BYTES], int const clen) = 0;
};

}

}

#endif
