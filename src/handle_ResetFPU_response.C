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
#include "ethercan/response_handlers/handle_ResetFPU_response.h"
#include "ethercan/time_utils.h"
#include "ethercan/FPUArray.h"


#ifdef DEBUG
#include <stdio.h>
#endif

namespace mpifps
{

namespace ethercanif
{

void handle_ResetFPU_response(const EtherCANInterfaceConfig&config,
			      const int fpu_id,
			      t_fpu_state& fpu,
			      int &count_pending,
			      const t_response_buf&data,
			      const int blen, TimeOutList&  timeout_list,
			      const E_CAN_COMMAND cmd_id,
			      const uint8_t sequence_number)
{
    assert(blen == 8);
    const E_MOC_ERRCODE response_errcode = static_cast<E_MOC_ERRCODE>((data[3] & 0xF0) >> 4);
    
    // clear pending time-out for reset command
    remove_pending(config, fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending, sequence_number);
    
    if (response_errcode == 0)
    {
        initialize_fpu(fpu);
	update_status_flags(fpu, UPDATE_FIELDS_DEFAULT, data);
	fpu.direction_alpha = DIRST_UNKNOWN;
	fpu.direction_beta = DIRST_UNKNOWN;

        if (fpu.pending_command_set != 0)
        {
            // remove *all* other pending commands
            for(int cmd_code=0; cmd_code < NUM_CAN_COMMANDS; cmd_code++)
            {
                if (((fpu.pending_command_set >> cmd_code) & 1) == 1)
                {
                    const E_CAN_COMMAND can_cmd = static_cast<E_CAN_COMMAND>(cmd_code);
                    remove_pending(config, fpu, fpu_id,  can_cmd, response_errcode,
                                   timeout_list, count_pending, sequence_number);
                }

            }
        }
    }
    fpu.ping_ok = true; // we known the current step counter

    // in protocol version 1, we do not know the last movement direction

}

}

}
