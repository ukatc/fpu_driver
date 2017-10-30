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
    UNINITIALISED       = 1,
    DATUM_SEARCH        = 2,
    INITIALISED         = 3,
    LOADING             = 4,
    READY_FORWARD       = 5,
    READY_BACKWARD      = 6,
    MOVING              = 7,
    FINISHED            = 8,
    ABORTED             = 9,

} ;


enum E_WaitTarget
{
    INITIALISED       = 1,
    AT_DATUM          = 2,
    READY_TO_MOVE     = 3,
    MOVEMENT_FINISHED = 4,
    MOVEMENT_ABORTED  = 5,

    ANY_CHANGE        = 6,
} ;



}
