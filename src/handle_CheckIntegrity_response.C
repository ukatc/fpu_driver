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
#include "ethercan/response_handlers/handle_CheckIntegrity_response.h"
#include "ethercan/time_utils.h"
#include "ethercan/FPUArray.h"


#ifdef DEBUG
#include <stdio.h>
#endif

namespace mpifps
{

namespace ethercanif
{

void handle_CheckIntegrity_response(const EtherCANInterfaceConfig&config,
                                    const int fpu_id,
                                    t_fpu_state& fpu,
                                    unsigned int &count_pending,
                                    const t_response_buf&data,
                                    const int blen, TimeOutList&  timeout_list,
                                    const E_CAN_COMMAND cmd_id,
                                    const uint8_t sequence_number)
{
    assert(blen == 8);
    const E_MOC_ERRCODE response_errcode = update_status_flags(fpu, UPDATE_FIELDS_NOSTEPS, data);

    // clear time-out flag
    remove_pending(config, fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending, sequence_number);

    if (response_errcode == 0)
    {
        fpu.ping_ok = true;
        fpu.crc32 =  (static_cast<uint32_t>(data[4])
                      | static_cast<uint32_t>(data[5] << 8)
                      | static_cast<uint32_t>(data[6] << 16)
                      | static_cast<uint32_t>(data[7] << 24));

        LOG_RX(LOG_INFO, "%18.6f : RX : "
               "checkIntegrity command for FPU %i : result 0X%04x\n",
               get_realtime(),
               fpu_id, fpu.crc32);
    }
    else
    {
        fpu.ping_ok = false;

        LOG_RX(LOG_ERROR, "%18.6f : RX : "
               "checkIntegrity command failed for FPU %i (errcode=%u)\n",
               get_realtime(),
               fpu_id, (unsigned int)response_errcode);
    }


}

}

}
