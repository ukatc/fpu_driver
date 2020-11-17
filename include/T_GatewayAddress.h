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
// NAME T_GatewayAddress.h
//
// This header defines a struct which holds the gateway address
//
////////////////////////////////////////////////////////////////////////////////

#ifndef T_GATEWAY_ADDRESS_H
#define T_GATEWAY_ADDRESS_H

#include <cstdint>

namespace mpifps
{

typedef struct t_gateway_address
{
    const char * ip;
    uint16_t port;
} t_gateway_address;

}

#endif
