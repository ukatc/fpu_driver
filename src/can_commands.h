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

enum E_CAN_COMMANDS
{
    // external accessible commands
    resetFPU           = 1,
    findDatum          = 2,
    lockFPU            = 3,
    unlockFPU          = 4,
    assignPosition     = 5,
    configMotion       = 6,
    reverseMotion      = 7,
    repeatMotion       = 8,
    executeMotion      = 9,
    abortMotion        = 10,
    untangleFPU        = 11,
    clearCollision     = 12,
    recoverCoordinates = 13,

    // internal commands
    moveUntilDatumGoesOn  = 100,
    moveUntilDatumGoesOff = 101,


};

enum E_FPU_STATE
{
    UNINITIALISED       = 1,
    LOCKED              = 2,
    COORDINATE_RECOVERY = 3,
    DATUM_SEARCH        = 4,
    INITIALISED         = 5,
    LOADING             = 6,
    READY_FORWARD       = 7,
    READY_BACKWARD      = 8,
    FINISHED            = 9,
    MOVING              = 10,
    ABORTED             = 11,
    COLLISION_DETECTED  = 12,
    LIMIT_STOP          = 13,
    COLLISION_RECOVERY  = 14,

} ;




