
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
// NAME E_CAN_COMMAND.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////
#ifndef E_CAN_COMMAND_H

#define E_CAN_COMMAND_H

namespace mpifps
{

enum E_CAN_COMMAND
{
    CCMD_NO_COMMAND        = 0,
    CCMD_PING_FPU          = 1,
    CCMD_MOVE_DATUM_OFF    = 2,
    CCMD_MOVE_DATUM_ON     = 3,
    CCMD_CONFIG_MOTION     = 4,
    CCMD_EXECUTE_MOTION    = 5,
    CCMD_REPEAT_MOTION     = 6,
    CCMD_REVERSE_MOTION    = 7,
    CCMD_REQUEST_STATUS    = 8,
    CCMD_REPORT_POSITIONS  = 9,
    CCMD_ASSIGN_POSITION   = 10,
    CCMD_ABORT_MOTION      = 11,
    CCMD_UNTANGLE_FPU      = 12,
    CCMD_CLEAR_COLLISION   = 13,
    CCMD_CHECK_INTEGRITY   = 14,
    CCMD_RESET_FPU         = 15,
    CCMD_LOCK_UNIT         = 16,
    CCMD_UNLOCK_UNIT       = 17,

    NUM_CAN_COMMANDS  = 18,
};


enum E_CAN_RESPONSE
{
    CRSP_BROADCAST         = 0,
    CRSP_PING_RESPONSE     = 1,
};


} // end of namespace

#endif
