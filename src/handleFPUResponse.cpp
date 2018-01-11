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

#include <pthread.h>

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

// logs error status in CAN response
  void logErrorStatus(int fpu_id, timespec time_stamp, int err_code)
{
  const char * err_msg = nullptr;
  switch (err_code)
    {
    case 0:
      err_msg = nullptr;
      break;

      

    case ER_COLLIDE  : err_msg = "FPU collision detected"                       ; break;
    case ER_INVALID  : err_msg = "received command not valid"                   ; break;
    case ER_WAVENRDY : err_msg = "waveform not ready"                           ; break;
    case ER_WAVE2BIG : err_msg = "waveform exceeds memory allocation"           ; break;
    case ER_TIMING   : err_msg = "step timing error (interrupt race condition)" ; break;
    case ER_M1LIMIT  : err_msg = "M1 Limit switch breached"                     ; break;
    case ER_PARAM    : err_msg = "parameter out of range"                       ; break;

    default:
    case ER_STALLX           :
    case ER_STALLY           :
    case ER_M2LIMIT:      err_msg = "obsolete error code received";      break;
      
    }
#ifdef DEBUG
  // FIXME: In production code, the logging should be taken out of
  // the time-critical path so that even a large amount of log messages
  // will not affect the responsiveness of the driver.
  printf("[%lu.%lu]: FPU #%04i %s\n",
         time_stamp.tv_sec, time_stamp.tv_nsec,
         fpu_id, err_msg);
#endif
}


void update_steps(int &alpha_steps, int &beta_steps, const t_response_buf& data)
{
    alpha_steps = (data[4] << 8) | data[5];
    beta_steps = (data[6] << 8) | data[7];
}

void handleFPUResponse(int fpu_id, t_fpu_state& fpu,
                       const t_response_buf& data,
                       const int blen, bool& clear_pending)
{
    uint8_t cmd_id = data[1];
    uint8_t response_status = data[2];
    uint8_t response_errcode = data[3] ? data[4] : 0;
    timespec cur_time;

    assert(blen == 8);
    
    get_monotonic_time(cur_time);
    
    switch (cmd_id)
    {
    case CCMD_CONFIG_MOTION   :
        // clear time-out flag
        clear_pending = true;
        if (response_errcode != 0)
        {
            logErrorStatus(fpu_id, cur_time, response_errcode);
            // if the FPU was in loading state, it is switched to RESTING,
            // otherwise unchanged.
            if (fpu.state == FPST_LOADING)
            {
                fpu.state = FPST_RESTING;
            }
        }
        else
        {
            if (response_status == STBT_WAVE_READY)
            {
                fpu.state = FPST_READY_FORWARD;
            }
            else
            {
                fpu.state = FPST_LOADING;
            }
        }
        if (fpu.pending_command == CCMD_CONFIG_MOTION)
        {
            fpu.last_command = fpu.pending_command;
            fpu.pending_command = CCMD_NO_COMMAND;
        }
        fpu.last_updated = cur_time;
        break;
        
    case CCMD_EXECUTE_MOTION  :
        // we do not clear the time-out flag now, but rather
        // wait for CMSG_FINISHED_MOTION for that.
        if (response_errcode == 0)
        {
            // FIXME: Update step counter in protocol version 2
            // update_steps(fpu.alpha_steps, fpu.beta_steps, data);
            fpu.state = FPST_MOVING;
            // status byte should show RUNNING_WAVE, too
        }
        fpu.last_updated = cur_time;
        break;

    case CMSG_FINISHED_MOTION:
        // clear time-out flag
        clear_pending = true;
        if (response_errcode == 0)
        {
            // FIXME: Update step counter in protocol version 2
            // update_steps(fpu.alpha_steps, fpu.beta_steps, data);
            fpu.state = FPST_RESTING;
        }
        if (fpu.pending_command == CCMD_EXECUTE_MOTION)
        {
            fpu.last_command = fpu.pending_command;
            fpu.pending_command = CCMD_NO_COMMAND;
        }
        fpu.last_updated = cur_time;
        break;
        
    case CCMD_ABORT_MOTION    :  
        // clear time-out flag
        clear_pending = true;
        if (response_errcode == 0)
        {
            // FIXME: Update step counter in protocol version 2
            //update_steps(fpu.alpha_steps, fpu.beta_steps, data);
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
        // clear time-out flag
        clear_pending = true;
        if (response_errcode == 0)
        {
            int asteps = (((data[6] << 8) | data[5]) << 8 | data[4]);
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
        // clear time-out flag
        clear_pending = true;
        if (response_errcode == 0)
        {
            int bsteps = (((data[6] << 8) | data[5]) << 8 | data[4]);
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
        // clear time-out flag
        clear_pending = true;
        if (fpu.pending_command == CCMD_PING_FPU)
        {
            fpu.last_command = fpu.pending_command;
            fpu.pending_command = CCMD_NO_COMMAND;
        }
        fpu.last_updated = cur_time;
        break;

    case CCMD_RESET_FPU       :  
        // clear time-out flag
        clear_pending = true;
        if (fpu.pending_command == CCMD_RESET_FPU)
        {
            fpu.last_command = fpu.pending_command;
            fpu.pending_command = CCMD_NO_COMMAND;
        }
        if (response_errcode == 0)
        {
            fpu.state = FPST_UNINITIALIZED;
        }
        fpu.last_updated = cur_time;
        break;
        
    case CCMD_FIND_DATUM :
        // we do not clear the time-out entry, because
        // we wait for the datum search to finish
        if (response_errcode == 0)
        {
            // datum search was successfully started
            fpu.state = FPST_DATUM_SEARCH;
        }
        // we leave findDatum as pending command, because
        // we have to wait for the final response.
        fpu.last_updated = cur_time;
        break;
    case CMSG_FINISHED_DATUM :  
        // clear time-out flag
        clear_pending = true;
        if (response_errcode != 0)
        {
            if (fpu.state == FPST_DATUM_SEARCH)
            {
                fpu.state = FPST_UNINITIALIZED;
            }
        }
        else
        {

            fpu.was_zeroed = true;
            fpu.alpha_steps = 0;
            fpu.beta_steps = 0;
            fpu.state = FPST_AT_DATUM;
        }
        
        if (fpu.pending_command == CCMD_FIND_DATUM)
        {
            fpu.last_command = fpu.pending_command;
            fpu.pending_command = CCMD_NO_COMMAND;
        }
        fpu.last_updated = cur_time;
        break;
    case CMSG_WARN_COLLISION_BETA:
        
        fpu.state = FPST_OBSTACLE_ERROR;
        fpu.beta_collision = true;
        // FIXME: Update step counter in protocol version 2
        //update_steps(fpu.alpha_steps, fpu.beta_steps, data);
        
        if (fpu.pending_command != CCMD_NO_COMMAND)
        {
            // clear time-out flag
            clear_pending = true;
        
            fpu.last_command = fpu.pending_command;
            fpu.pending_command = CCMD_NO_COMMAND;
        }
        fpu.last_updated = cur_time;
        break;
        
    case CMSG_WARN_LIMIT_ALPHA:
        fpu.state = FPST_OBSTACLE_ERROR;
        fpu.at_alpha_limit = true;
        // FIXME: Update step counter in protocol version 2
        //update_steps(fpu.alpha_steps, fpu.beta_steps, data);
        
        if (fpu.pending_command != CCMD_NO_COMMAND)
        {
            // clear time-out flag
            clear_pending = true;
        
            fpu.last_command = fpu.pending_command;
            fpu.pending_command = CCMD_NO_COMMAND;
        }
        fpu.last_updated = cur_time;
        break;


    case CCMD_ENABLE_BETA_COLLISION_PROTECTION:
        if (response_errcode != 0)
        {
            fpu.state = FPST_OBSTACLE_ERROR;
            fpu.beta_collision = true;
        }
        else
        {
            fpu.state = FPST_RESTING;
            fpu.beta_collision = false;
        }
        // FIXME: Update step counter in protocol version 2
        //update_steps(fpu.alpha_steps, fpu.beta_steps, data);
        
        if (fpu.pending_command != CCMD_NO_COMMAND)
        {
            // clear time-out flag
            clear_pending = true;
        
            fpu.last_command = fpu.pending_command;
            fpu.pending_command = CCMD_NO_COMMAND;
        }
        fpu.last_updated = cur_time;
        break;
        
    case CCMD_FREE_BETA_COLLISION:
        fpu.state = FPST_OBSTACLE_ERROR;
        // FIXME: Update step counter in protocol version 2
        //update_steps(fpu.alpha_steps, fpu.beta_steps, data);
        
        if (fpu.pending_command != CCMD_NO_COMMAND)
        {
            // clear time-out flag
            clear_pending = true;
        
            fpu.last_command = fpu.pending_command;
            fpu.pending_command = CCMD_NO_COMMAND;
        }
        fpu.last_updated = cur_time;
        break;
        
    case CCMD_NO_COMMAND      :
    default:
        // invalid command, ignore 
        // FIXME: log invalid responses
        break;

    }

}

}

}
