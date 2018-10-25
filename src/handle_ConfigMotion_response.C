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
#include "ethercan/response_handlers/handle_ConfigMotion_response.h"
#include "ethercan/time_utils.h"
#include "ethercan/FPUArray.h"


#ifdef DEBUG
#include <stdio.h>
#endif

namespace mpifps
{

namespace ethercanif
{

handle_ConfigMotion_response(const EtherCANInterfaceConfig&config,
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
    LOG_RX(LOG_TRACE_CAN_MESSAGES, "%18.6f : RX : handle_ConfigMotion:"
           " fpu #%u, segment %u: status=%u, error=%u\n",
           get_realtime(),
           fpu_id, fpu.num_waveform_segments,
           response_status, response_errcode);

    // clear time-out flag
    remove_pending(config, fpu, fpu_id, cmd_id, response_errcode, timeout_list, count_pending);
    if (response_errcode != 0)
    {
        logErrorStatus(config, fpu_id, response_errcode);
        // if the FPU was in loading state, it is switched to RESTING,
        // otherwise unchanged.

        // FIXME: decrease log level in production system to keep responsivity at maximum
        LOG_RX(LOG_ERROR, "%18.6f : RX : "
               "configMotion command for FPU %i failed with error code %i\n",
               get_realtime(),
               fpu_id,
               response_errcode);

        if (fpu.state == FPST_LOADING)
        {
            fpu.state = FPST_RESTING;
        }
    }
    else
    {
#if (CAN_PROTOCOL_VERSION == 1)
        fpu.num_waveform_segments = data[4];
        // FIXME: needs to be set from response for protocol version 2
#endif

        if (response_status & STBT_WAVE_READY)
        {
            fpu.state = FPST_READY_FORWARD;
            fpu.waveform_valid = true;
            fpu.waveform_ready = true;
        }
        else
        {
            fpu.state = FPST_LOADING;
        }
    }

    fpu.last_updated = cur_time;

}

}

}
