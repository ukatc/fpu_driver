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
// NAME handle_EnableAlphaLimitProtection_response.C
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////


#include <cassert>


#include "FPUState.h"
#include "ethercan/response_handlers/handle_EnableAlphaLimitProtection_response.h"
#include "ethercan/time_utils.h"
#include "ethercan/FPUArray.h"


#ifdef DEBUG
#include <stdio.h>
#endif

namespace mpifps
{

namespace ethercanif
{

void handle_EnableAlphaLimitProtection_response(const EtherCANInterfaceConfig&config,
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
    remove_pending(config, fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending, sequence_number);

    if (response_errcode == 0)
    {
        fpu.ping_ok = true;

    }
    else
    {
        fpu.ping_ok = false;

        LOG_RX(LOG_ERROR, "%18.6f : RX : "
               "enableAlphaLimitProtection command failed for FPU %i (errcode=%u)\n",
               get_realtime(),
               fpu_id, (unsigned int)response_errcode);
    }


}

}

}
