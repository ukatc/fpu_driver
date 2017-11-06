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

    enum E_FPU_STATE
    {
        FPST_UNKNOWN             = 0,
        FPST_UNINITIALISED       = 1,
        FPST_LOCKED              = 2,
        FPST_COORDINATE_RECOVERY = 3,
        FPST_DATUM_SEARCH        = 4,
        FPST_ABOVE_DATUM         = 5,
        FPST_INITIALISED         = 6,
        FPST_LOADING             = 7,
        FPST_READY_FORWARD       = 8,
        FPST_READY_BACKWARD      = 9,
        FPST_MOVING              = 10,
        FPST_FINISHED            = 11,
        FPST_ABORTED             = 12,
        FPST_COLLISION_DETECTED  = 13,
        FPST_LIMIT_STOP          = 14,
        FPST_COLLISION_RECOVERY  = 15,

        NUM_FPU_STATES      = 16,

    } ;

}


