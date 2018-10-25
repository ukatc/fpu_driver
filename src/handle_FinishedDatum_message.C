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
#include "ethercan/response_handlers/handle_FinishedDatum_message.h"
#include "ethercan/time_utils.h"
#include "ethercan/FPUArray.h"


#ifdef DEBUG
#include <stdio.h>
#endif

namespace mpifps
{

namespace ethercanif
{

handle_FinishedDatum_message(const EtherCANInterfaceConfig&config,
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
    //printf("finished: datum search for FPU %i \n", fpu_id);
    remove_pending(config, fpu, fpu_id,  CCMD_FIND_DATUM, response_errcode, timeout_list, count_pending);

    if ((response_status & STBT_M1LIMIT) || (response_errcode == MCE_WARN_LIMIT_SWITCH_BREACH))
    {
        fpu.alpha_datum_switch_active = true;
        fpu.at_alpha_limit = true;
        fpu.state = FPST_OBSTACLE_ERROR;
        fpu.waveform_valid = false;
        fpu.alpha_was_zeroed = false;
        fpu.beta_was_zeroed = false;
        fpu.ping_ok = false;

        // FIXME: decrease log level in production system to keep responsivity at maximum
        LOG_RX(LOG_ERROR, "%18.6f : RX : "
               "while waiting for end of datum command:"
               "limit switch breach message received for FPU %i\n",
               get_realtime(),
               fpu_id);
    }
    else if ((response_errcode == MCE_WARN_COLLISION_DETECTED) || fpu.beta_collision)
    {
        fpu.beta_collision = true;
        fpu.state = FPST_OBSTACLE_ERROR;
        fpu.waveform_valid = false;
        fpu.alpha_was_zeroed = false;
        fpu.beta_was_zeroed = false;
        fpu.ping_ok = false;

        // FIXME: decrease log level in production system to keep responsivity at maximum
        LOG_RX(LOG_ERROR, "%18.6f : RX : "
               "while waiting for end of datum command:"
               "collision detection message received for FPU %i\n",
               get_realtime(),
               fpu_id);
    }
    else if (response_errcode == MCE_ERR_DATUM_TIME_OUT)
    {
        // The datum operation was timed-out by the firmware.  The
        // only way this can regularly happen for the alpha arm is
        // if the motor is not moving, or the alpha switch is
        // broken. In the latter case, the alpha gearbox is
        // possibly already destroyed.
        fpu.state = FPST_ABORTED;
        fpu.waveform_valid = false;
        fpu.alpha_was_zeroed = false;
        fpu.beta_was_zeroed = false;
        fpu.ping_ok = false;

        // FIXME: decrease log level in production system to keep responsivity at maximum
        LOG_RX(LOG_ERROR, "%18.6f : RX : "
               "while waiting for finishing datum command:"
               "hardware datum time-out message received for FPU %i\n",
               get_realtime(),
               fpu_id);

        LOG_CONSOLE(LOG_ERROR, "%18.6f : RX : "
                    "while waiting for finishing datum command:"
                    "hardware datum time-out message received for FPU %i\n"
                    "\a\a\aWARNING: HARDWARE DAMAGE LIKELY\n",
                    get_realtime(),
                    fpu_id);
    }
    else if (response_status & STBT_ABORT_WAVE)
    {
        if (fpu.state != FPST_OBSTACLE_ERROR)
        {
            fpu.state = FPST_ABORTED;
        }
        fpu.waveform_valid = false;
        fpu.ping_ok = false;

        LOG_RX(LOG_DEBUG, "%18.6f : RX : "
               "while waiting for end of datum command:"
               "movement abortion message received for FPU %i\n",
               get_realtime(),
               fpu_id);
    }
    else if (response_errcode == MCE_ERR_DATUM_ON_LIMIT_SWITCH)
    {
        if (fpu.state == FPST_DATUM_SEARCH)
        {
            fpu.state = FPST_UNINITIALIZED;
            fpu.alpha_was_zeroed = false;
            fpu.beta_was_zeroed = false;
            fpu.ping_ok = false;
        }
        LOG_RX(LOG_ERROR, "%18.6f : RX : "
               "datum request rejected for FPU %i, because alpha limit switch active\n",
               get_realtime(),
               fpu_id);
    }
    else if (response_errcode != 0)
    {
        if (fpu.state == FPST_DATUM_SEARCH)
        {
            fpu.state = FPST_UNINITIALIZED;
            fpu.alpha_was_zeroed = false;
            fpu.beta_was_zeroed = false;
            fpu.ping_ok = false;
        }
    }
    else
    {
        // response_errcode was 0 and no bad status flags were set

        uint8_t exclusion_flags = data[5];
        if ((exclusion_flags & DATUM_SKIP_ALPHA) == 0)
        {
            fpu.alpha_was_zeroed = true;
            fpu.alpha_steps = 0;
        }
        if ((exclusion_flags & DATUM_SKIP_BETA) == 0)
        {
            fpu.beta_was_zeroed = true;
            fpu.beta_steps = 0;
        }
        if (fpu.beta_was_zeroed && fpu.alpha_was_zeroed)
        {
            fpu.state = FPST_AT_DATUM;
            fpu.ping_ok = true;
        }
        else
        {
            // if only one arm was zeroed, the state still counts as
            // uninitialized (zeroing only one arm is used only
            // for testing and engineering, not for instrument operation).
            fpu.state = FPST_UNINITIALIZED;
        }
        // in protocol version 1, we do not know the last movement direction
        fpu.direction_alpha = DIRST_UNKNOWN;
        fpu.direction_beta = DIRST_UNKNOWN;
    }

    fpu.last_updated = cur_time;

}

}

}
