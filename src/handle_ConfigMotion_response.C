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

void handle_ConfigMotion_response(const EtherCANInterfaceConfig&config,
                                  const int fpu_id,
                                  t_fpu_state& fpu,
                                  int &count_pending,
                                  const t_response_buf&data,
                                  const int blen, TimeOutList&  timeout_list,
                                  const E_CAN_COMMAND cmd_id,
                                  const uint8_t sequence_number)
{
    // update status fields, but not step counts (they do not fit into the response)
    assert(blen == 5);
    const E_MOC_ERRCODE response_errcode = update_status_flags(fpu, UPDATE_FIELDS_NOSTEPS, data);

    LOG_RX(LOG_TRACE_CAN_MESSAGES, "%18.6f : RX : handle_ConfigMotion:"
           " fpu #%u, segment %u: status=%u\n",
           get_realtime(),
           fpu_id, fpu.num_waveform_segments,
           response_errcode);

    // clear time-out flag
    remove_pending(config, fpu, fpu_id, cmd_id, response_errcode, timeout_list, count_pending, sequence_number);
    if (response_errcode != 0)
    {
        logErrorStatus(config, fpu_id, response_errcode);
        fpu.num_waveform_segments = 0;
        fpu.waveform_status = static_cast<E_WAVEFORM_ERRCODE>(data[5]);

        // FIXME: decrease log level in production system
        LOG_RX(LOG_ERROR, "%18.6f : RX : "
               "configMotion command for FPU %i failed with error code %i\n",
               get_realtime(),
               fpu_id,
               response_errcode);
    }
    else
    {
        fpu.num_waveform_segments = data[4];
        fpu.waveform_status = WAVEFORM_OK;
    }

}

}

}
