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
#include "DriverConstants.h"
#include "FPUState.h"
#include "canlayer/E_CAN_COMMAND.h"
#include "DriverState.h"

namespace mpifps
{



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
    // which is fine. (Wrapping of unsigned integral
    // types does not cause undefined  behavior in C/C++.)
    unsigned long count_timeout;

    // number of commands awaiting a response.
    int count_pending;

    // number of queued commands
    int num_queued;


    // state of the driver itself
    E_DriverState driver_state;
} t_grid_state;

}

#endif
