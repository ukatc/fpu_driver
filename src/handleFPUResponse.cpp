// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
// ESO - VLT Project
//
// Copyright 2017 E.S.O,
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// Pablo Gutierrez 2017-07-22  created CAN client sample
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
#include "canlayer/handleFPUResponse.h"
#include "canlayer/time_utils.h"

#ifdef DEBUG
#include <stdio.h>
#endif

namespace mpifps
{

namespace canlayer
{

 
void handleFPUResponse(t_fpu_state& fpu,
                       const t_response_buf& data,
                       const int blen)
{
    uint8_t cmd_id = data[1];
    uint8_t response_status = data[2];
    uint8_t response_errcode = data[3];
    timespec cur_time;
    
    get_monotonic_time(cur_time);
    
    switch (cmd_id)
    {
    case CCMD_CONFIG_MOTION   :  
    case CCMD_EXECUTE_MOTION  :  
        if (response_errcode == 0)
        {
            if (blen < 8)
            {
                break;
            }
            int asteps = (data[4] << 8) | data[5];
            fpu.alpha_steps = asteps;
            int bsteps = (data[6] << 8) | data[7];
            fpu.beta_steps = bsteps;
            fpu.state = FPST_FINISHED;
        }
        if (fpu.pending_command == CCMD_EXECUTE_MOTION)
        {
            fpu.last_command = fpu.pending_command;
            fpu.pending_command = CCMD_NO_COMMAND;
        }
        fpu.last_updated = cur_time;
        break;
        
    case CCMD_ABORT_MOTION    :  
        if (response_errcode == 0)
        {
            if (blen < 8)
            {
                break;
            }
            int asteps = (data[4] << 8) | data[5];
            fpu.alpha_steps = asteps;
            int bsteps = (data[6] << 8) | data[7];
            fpu.beta_steps = bsteps;
            fpu.state = FPST_ABORTED;
        }
        if (fpu.pending_command == CCMD_ABORT_MOTION)
        {
            fpu.last_command = fpu.pending_command;
            fpu.pending_command = CCMD_NO_COMMAND;
        }
        fpu.last_updated = cur_time;
        break;
    case CCMD_GET_STEPS_ALPHA :  
        if (response_errcode == 0)
        {
            if (blen < 6)
            {
                break;
            }
            int asteps = (((data[4] << 8) | data[5]) << 8 | data[6]);
#ifdef DEBUG
            printf("updating FPU.alpha_steps from %i to %i", fpu.alpha_steps, asteps);
                   
#endif
            fpu.alpha_steps = asteps;
        }
        if (fpu.pending_command == CCMD_GET_STEPS_ALPHA)
        {
            fpu.last_command = fpu.pending_command;
            fpu.pending_command = CCMD_NO_COMMAND;
        }
        fpu.last_updated = cur_time;
        break;
        
    case CCMD_GET_STEPS_BETA  :  
        if (response_errcode == 0)
        {
            if (blen < 6)
            {
                break;
            }
            int bsteps = (((data[4] << 8) | data[5]) << 8 | data[6]);
#ifdef DEBUG
            printf("updating FPU.beta_steps from %i to %i", fpu.beta_steps, bsteps);
                   
#endif
            fpu.beta_steps = bsteps;
        }
        if (fpu.pending_command == CCMD_GET_STEPS_BETA)
        {
            fpu.last_command = fpu.pending_command;
            fpu.pending_command = CCMD_NO_COMMAND;
        }
        fpu.last_updated = cur_time;
        break;
        
    case CCMD_PING_FPU        :
        if (response_errcode == 0)
        {
            fpu.ping_ok = true;
        }
        if (fpu.pending_command == CCMD_PING_FPU)
        {
            fpu.last_command = fpu.pending_command;
            fpu.pending_command = CCMD_NO_COMMAND;
        }
        fpu.last_updated = cur_time;
        break;

    case CCMD_RESET_FPU       :  
    case CCMD_AUTO_MOVE_DATUM :  
        if (response_errcode == 0)
        {

            fpu.is_initialized = true;
            fpu.alpha_steps = 0;
            fpu.beta_steps = 0;
            fpu.state = static_cast<E_FPU_STATE>(response_status);
        }
        
        if (fpu.pending_command == CCMD_AUTO_MOVE_DATUM)
        {
            fpu.last_command = fpu.pending_command;
            fpu.pending_command = CCMD_NO_COMMAND;
        }
        fpu.last_updated = cur_time;
        break;
    case CCMD_NO_COMMAND      :
    default:
        // invalid command, ignore for now
        // FIXME: log invalid responses
        break;

    }

}

}

}
