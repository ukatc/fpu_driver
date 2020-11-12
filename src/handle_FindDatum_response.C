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
// NAME handle_FindDatum_response.C
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////


#include <cassert>


#include "FPUState.h"
#include "ethercan/response_handlers/handle_FindDatum_response.h"
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

void handle_FindDatum_response(const EtherCANInterfaceConfig&config,
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

    // we do not clear the pending flag, because
    // we wait for the datum search to finish
    if (response_errcode == 0)
    {
        fpu.ping_ok = false;

        // As an edge case, it is possible that the command has
        // already been removed by a time-out handler. In that
        // case, re-add it as pending to avoid a stuck state.
        if (((fpu.pending_command_set >> CCMD_FIND_DATUM) & 1) == 0)
        {
            LOG_RX(LOG_ERROR, "%18.6f : RX : "
                   "FPU #%i: WARNING: findDatum was removed from pending set, added again\n",
                   get_realtime(),
                   fpu_id);

            const timespec new_timeout = {40, 0};
            add_pending(fpu, fpu_id, CCMD_FIND_DATUM,
                        new_timeout,
                        timeout_list, count_pending, 0);
        }

        // we leave findDatum as pending command, because
        // we have to wait for the final response.
    }
    else if (response_errcode == MCE_ERR_DATUM_ON_LIMIT_SWITCH)
    {
        remove_pending(config, fpu, fpu_id,  CCMD_FIND_DATUM, response_errcode, timeout_list, count_pending, sequence_number);

        fpu.alpha_was_referenced = false;
        fpu.beta_was_referenced = false;
        fpu.ping_ok = false;

        LOG_RX(LOG_ERROR, "%18.6f : RX : "
               "findDatum request rejected for FPU %i, because alpha limit switch active\n",
               get_realtime(),
               fpu_id);
    }
    else if (response_errcode == MCE_ERR_AUTO_DATUM_UNINITIALIZED)
    {
        remove_pending(config, fpu, fpu_id,  CCMD_FIND_DATUM, response_errcode, timeout_list, count_pending, sequence_number);

        fpu.alpha_was_referenced = false;
        fpu.beta_was_referenced = false;

        LOG_RX(LOG_ERROR, "%18.6f : RX : "
               "error:"
               "FPU %i was not initialised, automatic datum search rejected\n",
               get_realtime(),
               fpu_id);

    }


}

#pragma GCC diagnostic pop
}

}
