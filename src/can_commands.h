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
    NoCommand             = 100,
    moveUntilDatumGoesOn  = 101,
    moveUntilDatumGoesOff = 102,


};





