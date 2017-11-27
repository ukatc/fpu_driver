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

#ifndef FPU_STATE_H
#define FPU_STATE_H

#include "canlayer/E_CAN_COMMAND.h"
#include <time.h>

namespace mpifps
{

using canlayer::E_CAN_COMMAND;
    
enum E_FPU_STATE
{
    FPST_UNKNOWN             = 0,
    FPST_UNINITIALIZED       = 1,
    FPST_LOCKED              = 2,
    FPST_COORDINATE_RECOVERY = 3,
    FPST_LEAVING_DATUM       = 4,
    FPST_ABOVE_DATUM         = 5,
    FPST_DATUM_SEARCH        = 6,
    FPST_AT_DATUM            = 7,
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

};


typedef struct t_fpu_state
{
    // these members are the individual values
    // reported by FPU responses
    E_FPU_STATE state;
    int alpha_steps;
    int beta_steps;
    bool is_initialized;
    bool on_alpha_datum;
    bool on_beta_datum;
    bool alpha_collision;
    bool at_alpha_limit;
    bool beta_collision;
    bool ping_ok;



    // id of any still running and incomplete command
    E_CAN_COMMAND pending_command;
    // time when any running command is considered timed out
    // Note: this time needs to use the monotonic linux system
    // clock so that leap seconds don't trigger bugs.
    timespec cmd_timeout;
    // number of minor time-outs which have
    // been observed for the last command.
    int8_t timeout_count;

    // id of last command that was issued but not completed.
    E_CAN_COMMAND last_command;

    // id of last command that was completed
    E_CAN_COMMAND completed_command;    

    timespec last_updated;

    bool operator==(const  t_fpu_state &a) const
        {
            return (*this) == a;
        }


} t_fpu_state;

}


#endif
 
