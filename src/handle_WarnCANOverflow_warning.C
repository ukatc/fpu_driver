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
#include "ethercan/response_handlers/handle_WarnCANOverflow_warning.h"
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

void handle_WarnCANOverflow_warning(const EtherCANInterfaceConfig&config,
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

    // the most likely situation in which an overflow response occurs is
    // when uploading new waveforms to the FPUs
    if ( ((fpu.pending_command_set >> CCMD_CONFIG_MOTION) & 1) == 1)
    {

        // clear time-out flag
        remove_pending(config, fpu, fpu_id, CCMD_CONFIG_MOTION, response_errcode, timeout_list, count_pending, sequence_number);

        if (fpu.state == FPST_LOADING)
        {
            fpu.state = FPST_RESTING;
        }

    }

    fpu.ping_ok = false;
    fpu.can_overflow_errcount++; // this unsigned counter can wrap around - that's intentional.

    {
        const char * msg = "(n/a)";
        if (response_errcode == MCE_ERR_CAN_OVERFLOW_HW)
        {
            msg = "(hardware overflow)";
        }
        else if (response_errcode == MCE_ERR_CAN_OVERFLOW_SW)
        {
            msg = "(software overflow)";
        }

        LOG_RX(LOG_ERROR, "%18.6f : RX : "
               "CMSG_WARN_CAN_OVERFLOW (buffer overflow in FPU firmware) message received for FPU %i %s\n",
               get_realtime(),
               fpu_id,
               msg);

        LOG_CONSOLE(LOG_ERROR, "%18.6f : RX : "
                    "CMSG_WARN_CAN_OVERFLOW (buffer overflow in FPU firmware) message received for FPU %i %s\n",
                    get_realtime(),
                    fpu_id,
                    msg);
    }

}

#pragma GCC diagnostic pop

}

}
