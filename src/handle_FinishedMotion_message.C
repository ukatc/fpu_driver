// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 UKRI. See file "LICENSE" for license information.
//
// ESO - VLT Project
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client sample
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME FPUState.cpp
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////


#include <cassert>


#include "FPUState.h"
#include "ethercan/response_handlers/handle_FinishedMotion_message.h"
#include "ethercan/time_utils.h"
#include "ethercan/FPUArray.h"


#ifdef DEBUG
#include <stdio.h>
#endif

namespace mpifps
{

namespace ethercanif
{

handle_FinishedMotion_message(const EtherCANInterfaceConfig&config,
                              const int fpu_id,
                              t_fpu_state& fpu,
                              int &count_pending
                              const t_response_buf&data,
                              const int blen, TimeOutList&  timeout_list,
                              const E_CAN_COMMAND cmd_id,
                              const uin8_t response_status,
                              const E_MOC_ERRCODE response_errcode,
                              const timespec& cur_time)
{
    // clear time-out flag
    remove_pending(config, fpu, fpu_id,  CCMD_EXECUTE_MOTION, response_errcode, timeout_list, count_pending);
    if ((response_errcode == MCE_WARN_COLLISION_DETECTED) || fpu.beta_collision)
    {
        fpu.state = FPST_OBSTACLE_ERROR;
        fpu.movement_complete = false;
        fpu.waveform_valid = false;
        fpu.beta_collision = true;
        fpu.waveform_valid = false;
        fpu.alpha_was_zeroed = false;
        fpu.beta_was_zeroed = false;
        fpu.ping_ok = false;

        // FIXME: decrease log level in production system to keep responsivity at maximum
        LOG_RX(LOG_ERROR, "%18.6f : RX : "
               "while waiting for finishedMotion: "
               "collision detected message received for FPU %i\n",
               get_realtime(),
               fpu_id);

        LOG_CONSOLE(LOG_VERBOSE, "%18.6f : RX : "
                    "FPU # %i: executeMotion command finished error status "
                    "'MCE_WARN_COLLISION_DETECTED (beta arm collision)'"
                    " movement aborted.\n",
                    get_realtime(),
                    fpu_id);
    }
    else if ((response_status & STBT_M1LIMIT) || (response_errcode == MCE_WARN_LIMIT_SWITCH_BREACH) || fpu.at_alpha_limit)
    {
        fpu.at_alpha_limit = true;
        fpu.state = FPST_OBSTACLE_ERROR;
        fpu.movement_complete = false;
        fpu.waveform_valid = false;
        fpu.alpha_was_zeroed = false;
        fpu.beta_was_zeroed = false;
        fpu.alpha_datum_switch_active = true;
        fpu.ping_ok = false;

        // FIXME: decrease log level in production system to keep responsivity at maximum
        LOG_RX(LOG_ERROR, "%18.6f : RX : "
               "while waiting for finishedMotion: "
               "limit switch breach message received for FPU %i\n",
               get_realtime(),
               fpu_id);

        LOG_CONSOLE(LOG_VERBOSE, "%18.6f : RX : "
                    "FPU # %i: executeMotion command finished error status "
                    "'MCE_WARN_LIMIT_SWITCH_BREACH' (alpha limit switch breach)"
                    " movement aborted.\n",
                    get_realtime(),
                    fpu_id);

    }
    else if (response_status & STBT_ABORT_WAVE)
    {
        if (fpu.state != FPST_OBSTACLE_ERROR)
        {
            fpu.state = FPST_ABORTED;
        }
        fpu.movement_complete = false;
        fpu.waveform_valid = false;
        fpu.ping_ok = false;

        LOG_RX(LOG_ERROR, "%18.6f : RX : "
               "FPU # %i: executeMotion command finished error status 'FPST_ABORTED'"
               " movement aborted.\n",
               get_realtime(),
               fpu_id);

        LOG_CONSOLE(LOG_VERBOSE, "%18.6f : RX : "
                    "FPU # %i: executeMotion command finished error status 'FPST_ABORTED'"
                    " movement aborted.\n",
                    get_realtime(),
                    fpu_id);
    }
    else if (response_errcode == MCE_WARN_STEP_TIMING_ERROR)
    {
        // we got a step timing error (meaning the interrupt
        // handler running on the FPUs microcontroller could not
        // compute the step frequency quick enough for the
        // configured microstepping level).
        //
        // FIXME: This should possibly generate an abortMotion
        // message for all FPUs, because other FPUs can crash into
        // the stopped one if they continue moving.

        LOG_RX(LOG_ERROR, "%18.6f : RX : "
               "while waiting for finishedMotion: "
               "step timing error response received for FPU %i\n",
               get_realtime(),
               fpu_id);

        LOG_CONSOLE(LOG_VERBOSE, "%18.6f : RX : "
                    "FPU # %i: executeMotion command finished error status "
                    "'MCE_WARN_STEP_TIMING_ERROR (step timing error / firmware error)'"
                    " movement aborted.\n",
                    get_realtime(),
                    fpu_id);

        if (fpu.state != FPST_OBSTACLE_ERROR)
        {
            fpu.state = FPST_ABORTED;
        }
        fpu.movement_complete = false;
        fpu.waveform_valid = false;
        fpu.step_timing_errcount++;
        fpu.ping_ok = false;

    }
    else if (response_errcode == 0)
    {
        // FIXME: Update step counter in protocol version 2
        // update_steps(fpu.alpha_steps, fpu.beta_steps, data);
        if ( (fpu.state != FPST_OBSTACLE_ERROR) && (fpu.state != FPST_ABORTED))
        {
            fpu.state = FPST_RESTING;
            fpu.movement_complete = true;
            fpu.ping_ok = false;
        }

        // in protocol version 1, we do not know the last movement direction
        fpu.direction_alpha = DIRST_UNKNOWN;
        fpu.direction_beta = DIRST_UNKNOWN;
    }
    fpu.last_updated = cur_time;

}

}

}
