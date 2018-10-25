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
#include "ethercan/response_handlers/handle_AbortMotion_response.h"
#include "ethercan/time_utils.h"
#include "ethercan/FPUArray.h"


#ifdef DEBUG
#include <stdio.h>
#endif

namespace mpifps
{

namespace ethercanif
{

handle_AbortMotion_response(const EtherCANInterfaceConfig&config,
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
    if (response_errcode == 0)
    {
        // FIXME: Update step counter in protocol version 2
        //update_steps(fpu.alpha_steps, fpu.beta_steps, data);
        if (fpu.state != FPST_OBSTACLE_ERROR)
        {
            fpu.state = FPST_ABORTED;
        }
        // remove executeMotion from pending commands
        switch(fpu.state)
        {
        case FPST_MOVING:
            remove_pending(config, fpu, fpu_id,  CCMD_EXECUTE_MOTION, response_errcode, timeout_list, count_pending);
            break;
        case FPST_DATUM_SEARCH:
            remove_pending(config, fpu, fpu_id,  CCMD_FIND_DATUM, response_errcode, timeout_list, count_pending);
            break;
        default:
            /* the other commands are not movements */
            break;
        }
    }
    remove_pending(config, fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending);
    fpu.last_updated = cur_time;
    fpu.ping_ok = false;

    // this is set to a low logging level because any moving FPU
    // will send this message
    LOG_RX(LOG_DEBUG, "%18.6f : RX : "
           "abortMotion message received for FPU %i\n",
           get_realtime(),
           fpu_id);


}

}

}
