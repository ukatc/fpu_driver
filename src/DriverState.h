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

namespace mpifps
{

enum E_DriverState
{
    // this does not describe states of
    // individual FPUs (for example, some
    // having a collision and the collective
    // state "FINISHED" is a valid state).
    UNINITIALISED  = 1,
    DATUM_SEARCH   = 2,
    INITIALISED    = 3,
    LOADING        = 4,
    READY_FORWARD  = 5,
    READY_BACKWARD = 6,
    MOVING         = 7,
    FINISHED       = 8,
    ABORTED        = 9,
    NO_CONNECTION  = 10,

} ;


// keep in mind that these target states
// describe desired collective states of the
// FPU grid - the waitForState() command will
// return early on errors but errors are not
// a desired target state.
enum E_WaitTarget
{
    INITIALISED   = 1,
    AT_DATUM      = 2,
    READY_TO_MOVE = 3,
    MOVEMENT_FINISHED = 4,

    // Note: Using this target requires much more
    // frequent signalling, this possibly
    // affects performance.
    ANY_CHANGE = 10,
} ;



}
