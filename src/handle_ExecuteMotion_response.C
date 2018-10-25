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
#include "ethercan/response_handlers/handle_ExecuteMotion_response.h"
#include "ethercan/time_utils.h"
#include "ethercan/FPUArray.h"


#ifdef DEBUG
#include <stdio.h>
#endif

namespace mpifps
{

namespace ethercanif
{

handle_ExcecuteMotion_response(const EtherCANInterfaceConfig&config,
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
    // we do not clear the time-out flag now, but rather
    // wait for CMSG_FINISHED_MOTION for that.
    if (response_errcode == 0)
    {
        // FIXME: Update step counter in protocol version 2
        // update_steps(fpu.alpha_steps, fpu.beta_steps, data);
        if ((fpu.state != FPST_OBSTACLE_ERROR) && (fpu.state != FPST_ABORTED))
        {
            fpu.state = FPST_MOVING;
        }
        fpu.movement_complete = false;;
        // status byte should show RUNNING_WAVE, too

        // in protocol version 1, we do not know the last movement direction
        fpu.direction_alpha = DIRST_UNKNOWN;
        fpu.direction_beta = DIRST_UNKNOWN;
        fpu.ping_ok = false;

        // As an edge case, if the confirmation arrives extremely
        // large, then it is possible that the command has already
        // been removed by a time-out handler. In that case,
        // re-add it as pending to avoid a stuck state.
        if (((fpu.pending_command_set >> CCMD_EXECUTE_MOTION) & 1) == 0)
        {
            LOG_RX(LOG_DEBUG, "%18.6f : RX : "
                   "FPU #%i: WARNING: executeMotion was removed from pending set (%0x), added again\n",
                   get_realtime(),
                   fpu_id, fpu.pending_command_set);

            const timespec new_timeout = {40, 0};
            add_pending(fpu, fpu_id, CCMD_EXECUTE_MOTION,
                        new_timeout,
                        timeout_list, count_pending, 0);
        }
    }
    else
    {
        // clear timeout status
        remove_pending(config, fpu, fpu_id,  CCMD_EXECUTE_MOTION, response_errcode,
                       timeout_list, count_pending);

        // FIXME: decrease log level in production system to keep responsivity at maximum
        LOG_RX(LOG_ERROR, "%18.6f : RX : "
               "FPU # %i: executeMotion command got error response code #%i,"
               " removed from pending list.\n",
               get_realtime(),
               fpu_id,
               response_errcode);

        if ((response_errcode == MCE_ERR_WAVEFORM_NOT_READY)
                || (response_errcode == MCE_ERR_INVALID_PARAMETER))
        {
            if ((fpu.state == FPST_READY_FORWARD)
                    || (fpu.state == FPST_READY_REVERSE))
            {
                fpu.state = FPST_RESTING;
            }
            fpu.waveform_valid = false;

            // This message is set to a low log level because it is sent by any extra FPUs
            // which have not been configured
            LOG_CONSOLE(LOG_DEBUG, "%18.6f : RX : "
                        "FPU # %i: executeMotion command got error response 'ER_WAVENREDY' / 'MCE_ERR_INVALID_PARAMETER'"
                        " command skipped.\n",
                        get_realtime(),
                        fpu_id);
        }
        else if ((response_status & STBT_M1LIMIT) || (response_errcode == MCE_WARN_LIMIT_SWITCH_BREACH))
        {
            fpu.state = FPST_OBSTACLE_ERROR;
            fpu.waveform_valid = false;
            fpu.at_alpha_limit = true;
            fpu.ping_ok = false;

            LOG_CONSOLE(LOG_ERROR, "%18.6f : RX : "
                        "FPU # %i: executeMotion command got error status 'MCE_WARN_LIMIT_SWITCH_BREACH'/'STBT_M1LIMIT'"
                        " command cancelled.\n",
                        get_realtime(),
                        fpu_id);

        }
        else if (response_errcode == MCE_WARN_COLLISION_DETECTED)
        {
            fpu.alpha_datum_switch_active = true;
            fpu.state = FPST_OBSTACLE_ERROR;
            fpu.waveform_valid = false;

            LOG_CONSOLE(LOG_ERROR, "%18.6f : RX : "
                        "FPU # %i: executeMotion command got error response code 'MCE_WARN_COLLISION_DETECTED'"
                        " command cancelled.\n",
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

            LOG_CONSOLE(LOG_ERROR, "%18.6f : RX : "
                        "FPU # %i: executeMotion command got error status 'STBT_ABORT_WAVE'"
                        " command cancelled.\n",
                        get_realtime(),
                        fpu_id);
        }

    }
    fpu.last_updated = cur_time;
}

}

}
