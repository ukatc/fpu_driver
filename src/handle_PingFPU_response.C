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
#include "ethercan/response_handlers/handle_PingFPU_response.h"
#include "ethercan/time_utils.h"
#include "ethercan/FPUArray.h"


#ifdef DEBUG
#include <stdio.h>
#endif

namespace mpifps
{

namespace ethercanif
{

handle_PingFPU_response(const EtherCANInterfaceConfig&config,
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
    remove_pending(config, fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending);
    fpu.last_updated = cur_time;
    if (fpu.state == FPST_UNKNOWN)
    {
        fpu.state = FPST_UNINITIALIZED;
    }

    if (response_errcode == 0)
    {
        const uint16_t asteps_coded = (data[5] << 8) |  data[4];
        const uint16_t bsteps_coded = (data[7] << 8) |  data[6];
        const int asteps = unfold_stepcount_alpha(asteps_coded);
        const int bsteps = unfold_stepcount_beta(bsteps_coded);

        fpu.alpha_steps = asteps;
        fpu.beta_steps = bsteps;
        fpu.ping_ok = true;

#if 0
        // In protocol version 1, we (mis)use a ping after
        // a repeatMotion to retrieve the FPU state
        // and switch to READY_* again.
        if ((fpu.waveform_valid)
                && (fpu.waveform_ready)
                && (fpu.state = FPST_RESTING))
        {
            if (fpu.waveform_reversed)
            {
                fpu.state = FPST_READY_REVERSE;
            }
            else
            {
                fpu.state = FPST_READY_FORWARD;
            }
        }
#endif
    }
    else
    {
        fpu.ping_ok = false;

        // FIXME: decrease log level in production system to keep responsivity at maximum
        LOG_RX(LOG_ERROR, "%18.6f : RX : "
               "pingFPU command failed for FPU %i\n",
               get_realtime(),
               fpu_id);
    }


}

}

}
