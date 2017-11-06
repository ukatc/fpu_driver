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
// NAME FPU_CAN_driver.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////


enum E_FPU_STATE
{
    UNKNOWN             = 0,
    UNINITIALISED       = 1,
    LOCKED              = 2,
    COORDINATE_RECOVERY = 3,
    DATUM_SEARCH        = 4,
    ABOVE_DATUM         = 5,
    INITIALISED         = 6,
    LOADING             = 7,
    READY_FORWARD       = 8,
    READY_BACKWARD      = 9,
    MOVING              = 10,
    FINISHED            = 11,
    ABORTED             = 12,
    COLLISION_DETECTED  = 13,
    LIMIT_STOP          = 14,
    COLLISION_RECOVERY  = 15,

    NUM_FPU_STATES      = 16,

} ;




