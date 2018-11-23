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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

void handle_FinishedDatum_message(const EtherCANInterfaceConfig&config,
                                  const int fpu_id,
                                  t_fpu_state& fpu,
                                  int &count_pending,
                                  const t_response_buf&data,
                                  const int blen, TimeOutList&  timeout_list,
                                  const E_CAN_COMMAND cmd_id,
                                  const uint8_t sequence_number)
{
    // the error code carries an extra value if only the alpha or only the beta
    // arm was datumed.
    assert(blen == 8);
    const E_MOC_ERRCODE response_errcode = update_status_flags(fpu, UPDATE_FIELDS_DEFAULT, data);

    // clear time-out flag
    remove_pending(config, fpu, fpu_id,  CCMD_FIND_DATUM, response_errcode, timeout_list, count_pending, sequence_number);


    if ((fpu.at_alpha_limit) || (response_errcode == MCE_WARN_LIMIT_SWITCH_BREACH))
    {
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
        // The datum operation was timed-out by the firmware.
        // This can be due to broken FPU hardware,
        // such as a non-functioning datum switch.
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
    else if (fpu.state == FPST_ABORTED)
    {
        fpu.ping_ok = false;

        LOG_RX(LOG_DEBUG, "%18.6f : RX : "
               "while waiting for datum command:"
               "FPU %i is now in aborted state\n",
               get_realtime(),
               fpu_id);
    }
    else if (response_errcode == MCE_ERR_DATUM_ON_LIMIT_SWITCH)
    {
        if (fpu.state == FPST_DATUM_SEARCH)
        {
            fpu.alpha_was_zeroed = false;
            fpu.beta_was_zeroed = false;
            fpu.ping_ok = false;
        }
        LOG_RX(LOG_ERROR, "%18.6f : RX : "
               "datum request rejected for FPU %i, because alpha limit switch active\n",
               get_realtime(),
               fpu_id);
    }
    else if (!( (response_errcode ==  MCE_FPU_OK)
                || (response_errcode ==  MCE_NOTIFY_DATUM_ALPHA_ONLY)
                || (response_errcode ==  MCE_NOTIFY_DATUM_BETA_ONLY)))
    {
        fpu.alpha_was_zeroed = false;
        fpu.beta_was_zeroed = false;
        fpu.ping_ok = false;
    }
    else
    {
        // response_errcode was 0 and no bad status flags were set

        if ((response_errcode ==  MCE_FPU_OK)
                || (response_errcode ==  MCE_NOTIFY_DATUM_ALPHA_ONLY))
        {
            fpu.alpha_was_zeroed = true;
            fpu.alpha_steps = 0;
        }
        if ((response_errcode ==  MCE_FPU_OK)
                || (response_errcode ==  MCE_NOTIFY_DATUM_BETA_ONLY))
        {
            fpu.beta_was_zeroed = true;
            fpu.beta_steps = 0;
        }
        if (fpu.beta_was_zeroed && fpu.alpha_was_zeroed)
        {
            fpu.ping_ok = true;
        }
    }


}

#pragma GCC diagnostic pop
}

}
