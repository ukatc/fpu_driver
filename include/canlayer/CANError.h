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
// NAME CANError.h
//
// Defines low-level error states for CAN and
// socket operations.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef CAN_ERROR_H
#define CAN_ERROR_H

namespace mpifps
{

namespace canlayer
{

enum E_CANError
{
    UNINITIALIZED             = 1,
    MINOR_TIMEOUT             = 2,
    MAJOR_TIMEOUT             = 3,
    LOST_CONNECTION           = 4,
    SOCKET_CLOSED             = 5,

};

}

}
#endif





