// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
// ESO - VLT Project
//
// Copyright 2017 E.S.O,
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client sample
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME T_GridState.h
//
// This header defines a struct which holds the mirrored state
// if the whole FPU array
//
////////////////////////////////////////////////////////////////////////////////

#ifndef T_GATEWAY_ADDRESS_H
#define T_GATEWAY_ADDRESS_H

namespace mpifps
{

typedef struct t_gateway_address
{
    const char * ip;
    uint16_t port;
} t_gateway_address;

}

#endif
