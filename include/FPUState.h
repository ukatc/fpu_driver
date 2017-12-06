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
    FPST_UNKNOWN                 = 0,
    FPST_UNINITIALIZED           = 1,
    FPST_LOCKED                  = 2,
    FPST_DATUM_SEARCH            = 3,
    FPST_AT_DATUM                = 4,
    FPST_LOADING                 = 5,
    FPST_READY_FORWARD           = 6,
    FPST_READY_BACKWARD          = 7,
    FPST_MOVING                  = 8,
    FPST_FINISHED                = 9,
    FPST_ABORTED                 = 10,
    FPST_MOVEMENT_INTERRUPTED    = 11,

    NUM_FPU_STATES               = 12,

};

// known direction of movement
// (keep in mind that the command parameter
// is differently encoded)
enum E_MOVEMENT_DIRECTION
{
    DIRST_UNKNOWN          = 0,
    DIRST_ANTI_CLOCKWISE   = 3, // we could also use WYDDERSHINS here
    DIRST_CLOCKWISE        = 4, // and "DEOSIL" here which is from Gaelic
    DIRST_RESTING_LAST_CW  = 5, // might or might not be needed
    DIRST_RESTING_LAST_ACW = 6, // might or might not be needed
};



typedef struct t_fpu_state
{
    E_FPU_STATE state;
    // id of any still running and incomplete command
    E_CAN_COMMAND pending_command;
    // id of last command that was issued but not completed.
    E_CAN_COMMAND last_command;
    // id of last command that was completed
    E_CAN_COMMAND completed_command;    

    // time when any running command is considered timed out
    // Note: this time needs to use the monotonic linux system
    // clock so that leap seconds don't trigger bugs.
    timespec cmd_timeout;
    timespec last_updated;

    // these members are the individual values
    // reported by FPU responses
    int alpha_steps;
    int beta_steps;
    int num_waveforms;
    // number of minor time-outs which have
    // been observed for the last command.
    uint16_t timeout_count;
    E_MOVEMENT_DIRECTION direction_alpha;
    E_MOVEMENT_DIRECTION direction_beta;
    uint8_t sequence_number; // number of last pending / received command
    bool ping_ok;    // connection working
    bool was_zeroed; /* steps are validly calibrated by finding datum.
                        This is required for any science observatons. */
    bool is_locked;  // FPU was locked by operator
    bool alpha_datum_switch_active; // alpha datum switch is on
    bool beta_datum_switch_active; // beta datum switch is on
    bool at_alpha_limit; // alpha arm has reached limit (detected by datum off)
    bool beta_collision;
    bool waveform_valid; /* waveform completely loaded and not invalidated by collision
                            or abort message. */
    bool waveform_ready; // FPU can execute waveform
    bool waveform_reversed; // false means anti-clockwise for positive step numbers





    bool operator==(const  t_fpu_state &a) const
        {
            return (*this) == a;
        }


} t_fpu_state;

}


#endif
 
