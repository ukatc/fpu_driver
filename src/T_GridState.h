// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
// ESO - VLT Project
//
// Copyright 2017 E.S.O,
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client sample
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME T_GridState.h
//
// This header defines a struct which holds the mirrored state
// if the whole FPU array
//
////////////////////////////////////////////////////////////////////////////////

#ifndef T_GRID_STATE_H
#define T_GRID_STATE_H

#include <time.h>
#include <stdint.h>
#include "FPUState.h"
#include "E_CAN_COMMAND.h"
#include "DriverState.h"

namespace mpifps
{

typedef struct
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

} t_fpu_state;

typedef int t_counts[NUM_FPU_STATES];

typedef struct
{
    // individual states of each FPU. The index
    // is always the logical ID of each FPU.
    t_fpu_state FPU_state[MAX_NUM_POSITIONERS];

    // count of each FPU state
    t_counts Counts;


    // number of minor time-outs
    // Important: This unsigned counter wraps around
    // which is fine. (Wrapping of unsigned integer
    // types does not cause undefined  behavior in C.)
    unsigned long count_timeout;

    // number of commands awaiting a response.
    unsigned int count_pending;


    // so far unreported error
    E_DriverState driver_state;
} t_grid_state;

}

#endif
