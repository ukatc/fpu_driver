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
#include "ethercan/response_handlers/handle_WarnCollisionBeta_warning.h"
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


void handle_WarnCollisionBeta_warning(const EtherCANInterfaceConfig&config,
				      const int fpu_id,
				      t_fpu_state& fpu,
				      int &count_pending,
				      const t_response_buf&data,
				      const int blen, TimeOutList&  timeout_list,
				      const E_CAN_COMMAND cmd_id,
				      const uint8_t sequence_number)
{
    assert(blen == 8);
    const E_MOC_ERRCODE response_errcode = update_status_flags(fpu, UPDATE_FIELDS_DEFAULT, data);

    LOG_RX(LOG_ERROR, "%18.6f : RX : "
           "collision detection message received for FPU %i\n",
           get_realtime(),
           fpu_id);

    LOG_CONSOLE(LOG_ERROR, "%18.6f : RX : "
                "FPU # %i: beta arm collision detection message received.\n",
                get_realtime(),
                fpu_id);

    if (fpu.state == FPST_MOVING)
    {
        // clear time-out flag
        remove_pending(config, fpu, fpu_id,  CCMD_EXECUTE_MOTION, response_errcode, timeout_list, count_pending, sequence_number);

    }

    if (fpu.state == FPST_DATUM_SEARCH)
    {
        // clear time-out flag
        remove_pending(config, fpu, fpu_id,  CCMD_FIND_DATUM, response_errcode, timeout_list, count_pending, sequence_number);

    }

    fpu.waveform_valid = false;
    fpu.alpha_was_zeroed = false;
    fpu.beta_was_zeroed = false;
    fpu.ping_ok = false;


}
#pragma GCC diagnostic pop

}

}
