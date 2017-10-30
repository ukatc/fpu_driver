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
// NAME CANError.h
//
// Defines low-level error states for CAN and
// socket operations.
//
////////////////////////////////////////////////////////////////////////////////


enum E_CANError
{
    UNINITIALIZED             = 1,
    MINOR_TIMEOUT             = 2,
    MAJOR_TIMEOUT             = 3,
    LOST_CONNECTION           = 4,
    SOCKET_CLOSED             = 5,
    
} ;




