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
// NAME FPUState.h
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
        FPST_LEAVING_DATUM       = 4,
        FPST_ABOVE_DATUM         = 5,
        FPST_DATUM_SEARCH        = 6,
        FPST_INITIALISED         = 7,
        FPST_LOADING             = 8,
        FPST_READY_FORWARD       = 9,
        FPST_READY_BACKWARD      = 10,
        FPST_MOVING              = 11,
        FPST_FINISHED            = 12,
        FPST_ABORTED             = 13,
        FPST_COLLISION_DETECTED  = 14,
        FPST_LIMIT_STOP          = 15,
        FPST_COLLISION_RECOVERY  = 16,

        NUM_FPU_STATES      = 17,

    } ;

}


