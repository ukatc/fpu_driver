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
 
void handleFPUResponse(int fpu_id, t_fpu_state& fpu,
                       const t_response_buf& data,
                       const int blen)
{
    uint8_t cmd_id = data[1];
    uint8_t response_status = data[2];
    uint8_t response_errcode = data[3] ? data[4] : 0;
    timespec cur_time;
    
    get_monotonic_time(cur_time);
    
    switch (cmd_id)
    {
    case CCMD_CONFIG_MOTION   :
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
      break;
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
        if (response_errcode == 0)
        {
            if (blen < 6)
            {
                break;
            }
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
        if (fpu.pending_command == CCMD_PING_FPU)
        {
            fpu.last_command = fpu.pending_command;
            fpu.pending_command = CCMD_NO_COMMAND;
        }
        fpu.last_updated = cur_time;
        break;

    case CCMD_RESET_FPU       :  
    case CCMD_FIND_DATUM :  
        if (response_errcode == 0)
        {

            fpu.was_zeroed = true;
            fpu.alpha_steps = 0;
            fpu.beta_steps = 0;
            fpu.state = static_cast<E_FPU_STATE>(response_status);
        }
        
        if (fpu.pending_command == CCMD_FIND_DATUM)
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
