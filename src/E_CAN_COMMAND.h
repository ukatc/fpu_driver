
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
// NAME FPU_CAN_driver.h
// 
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

namespace mpifps {

    enum E_CAN_COMMAND
    {
        PING_FPU          = 1,
        MOVE_DATUM_OFF    = 2,
        MOVE_DATUM_ON     = 3,
        CONFIG_MOTION     = 4,
        EXECUTE_MOTION    = 5,
        REPEAT_MOTION     = 6,
        REVERSE_MOTION    = 7,
        REQUEST_STATUS    = 8,
        REPORT_POSITIONS  = 9,
        ASSIGN_POSITION   = 10,
        ABORT_MOTION      = 11,
        UNTANGLE_FPU      = 12,
        CLEAR_COLLISION   = 13,
        CHECK_INTEGRITY   = 14,
        RESET_FPU         = 15,
        LOCK_UNIT         = 16,
        UNLOCK_UNIT       = 17,
    }
    
 

} // end of namespace
