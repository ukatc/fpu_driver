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
#include "ethercan/decode_CAN_response.h"

#ifdef DEBUG
#include <stdio.h>
#endif

namespace mpifps
{

namespace ethercanif
{

void handle_AbortMotion_response(const EtherCANInterfaceConfig&config,
                                 const int fpu_id,
                                 t_fpu_state& fpu,
                                 unsigned int &count_pending,
                                 const t_response_buf&data,
                                 const int blen, TimeOutList&  timeout_list,
                                 const E_CAN_COMMAND cmd_id,
                                 const uint8_t sequence_number)
{

    assert(blen == 8);
    const E_MOC_ERRCODE response_errcode = update_status_flags(fpu, UPDATE_FIELDS_DEFAULT, data);

    // clear time-out flag
    if (response_errcode == MCE_FPU_OK)
    {
        if (fpu.state != FPST_OBSTACLE_ERROR)
        {
            fpu.state = FPST_ABORTED;
        }
        // remove executeMotion from pending commands
        switch(fpu.state)
        {
        case FPST_MOVING:
            remove_pending(config, fpu, fpu_id,  CCMD_EXECUTE_MOTION, response_errcode, timeout_list, count_pending, sequence_number);
            break;
        case FPST_DATUM_SEARCH:
            remove_pending(config, fpu, fpu_id,  CCMD_FIND_DATUM, response_errcode, timeout_list, count_pending, sequence_number);
            break;
        default:
            /* the other commands are not movements */
            break;
        }
    }
    remove_pending(config, fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending, sequence_number);
    fpu.ping_ok = false;

    // this is set to a low logging level because any moving FPU
    // will send this message when aborted
    LOG_RX(LOG_DEBUG, "%18.6f : RX : "
           "abortMotion message received for FPU %i\n",
           get_realtime(),
           fpu_id);


}

}

}
