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
#include "canlayer/FPUArray.h"


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
    const char * err_msg = "(no error)";
    switch (err_code)
    {
    case 0:
        err_msg = "(no error)";
        break;



    case ER_COLLIDE  :
        err_msg = "FPU collision detected"                       ;
        break;
    case ER_INVALID  :
        err_msg = "received command not valid"                   ;
        break;
    case ER_WAVENRDY :
        err_msg = "waveform not ready"                           ;
        break;
    case ER_WAVE2BIG :
        err_msg = "waveform exceeds memory allocation"           ;
        break;
    case ER_TIMING   :
        err_msg = "step timing error (interrupt race condition)" ;
        break;
    case ER_M1LIMIT  :
        err_msg = "M1 Limit switch breached"                     ;
        break;
    case ER_PARAM    :
        err_msg = "parameter out of range"                       ;
        break;

    default:
    case ER_STALLX           :
    case ER_STALLY           :
    case ER_M2LIMIT:
        err_msg = "obsolete error code received";
        break;

    }
#ifdef DEBUG
    // FIXME: In production code, logging of FPU error responses should
    // be taken out of the time-critical path so that even a large
    // amount of log messages will not affect the responsiveness of the
    // driver.

    printf("[%lu.%lu]: FPU #%04i : error response msg = %s\n",
           time_stamp.tv_sec, time_stamp.tv_nsec,
           fpu_id, err_msg);
#endif
}


//void update_steps(int &alpha_steps, int &beta_steps, const t_response_buf& data)
//{
//    alpha_steps = (data[4] << 8) | data[5];
//    beta_steps = (data[6] << 8) | data[7];
//}
//

// Takes an unsiged 16-bit value.
// Decodes step count as a 16-bit value
// with an asymmetric range.
int unfold_stepcount_alpha(const uint16_t step_count)
{
    int val = static_cast<int>(step_count);
    if (val >= 55000)
    {
        val -= (1 << 16);
    }
    return val;

}

// Takes an unsiged 16-bit value.
// Decodes step count as a signed 16-bit
// value with an symmetric range.
int unfold_stepcount_beta(const uint16_t step_count)
{
    int val = static_cast<int>(step_count);
    if (val >= (1 << 15))
    {
        val -= (1 << 16);
    }
    return val;

}

// Converts the response value for a datum search
// deviation into an integer. This is valid for both
// the alpha and the beta arm.
int unfold_steps_deviation(const uint16_t step_count)
{
    int val = static_cast<int>(step_count);
    if (val >= (1 << 15))
    {
        val -= (1 << 16);
    }
    return val;

}


void update_status_flags(t_fpu_state& fpu, unsigned int status_mask)
{
    fpu.waveform_ready = (status_mask & STBT_WAVE_READY) != 0;
    fpu.at_alpha_limit = (status_mask & STBT_M1LIMIT) != 0;
    fpu.waveform_reversed = (status_mask & STBT_REVERSE_WAVE) != 0;

}

void handleFPUResponse(int fpu_id, t_fpu_state& fpu,
                       const t_response_buf& data,
                       const int blen, TimeOutList& timeout_list,
                       int &count_pending)
{
    E_CAN_COMMAND cmd_id = static_cast<E_CAN_COMMAND>(data[1]);
    uint8_t response_status = data[2];
    update_status_flags(fpu, response_status);

    E_MOC_ERRCODE response_errcode = data[3] ? static_cast<E_MOC_ERRCODE>(data[4]) : ER_OK;
    timespec cur_time;

    assert(blen == 8);

    get_monotonic_time(cur_time);

    switch (cmd_id)
    {
    case CCMD_CONFIG_MOTION   :
      printf("handle_ConfigMotion: fpu #%i, segment %i: status=%i, error=%i\n",
	     fpu_id, fpu.num_waveform_segments,
	     response_status, response_errcode);
        // clear time-out flag
        remove_pending(fpu, fpu_id, cmd_id, response_errcode, timeout_list, count_pending);
        if (response_errcode != 0)
        {
            //logErrorStatus(fpu_id, cur_time, response_errcode);
            // if the FPU was in loading state, it is switched to RESTING,
            // otherwise unchanged.
            if (fpu.state == FPST_LOADING)
            {
                fpu.state = FPST_RESTING;
            }
        }
        else
        {
#if (CAN_PROTOCOL_VERSION == 1)
            fpu.num_waveform_segments = data[4];
            // FIXME: needs to be set from response for protocol version 2
#endif
            
            if (response_status & STBT_WAVE_READY)
            {
                fpu.state = FPST_READY_FORWARD;
                fpu.waveform_valid = true;
                fpu.waveform_ready = true;
            }
            else
            {
                fpu.state = FPST_LOADING;
            }
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
            fpu.movement_complete = false;;
            // status byte should show RUNNING_WAVE, too

            // in protocol version 1, we do not know the last movement direction
            fpu.direction_alpha = DIRST_UNKNOWN;
            fpu.direction_beta = DIRST_UNKNOWN;

            // As an edge case, if the confirmation arrives extremely
            // large, then it is possible that the command has already
            // been removed by a time-out handler. In that case,
            // re-add it as pending to avoid a stuck state.
            if (((fpu.pending_command_set >> CCMD_EXECUTE_MOTION) & 1) == 0)
            {
#ifdef DEBUG
                printf("FPU #%i: WARNING: executeMotion was removed from pending set (%0x), added again\n",
                       fpu_id, fpu.pending_command_set);
#endif
                const timespec new_timeout = {40, 0};
                add_pending(fpu, fpu_id, CCMD_EXECUTE_MOTION,
                            new_timeout,
                            timeout_list, count_pending);
            }
        }
        else
        {
            // clear timeout status
            remove_pending(fpu, fpu_id,  CCMD_EXECUTE_MOTION, response_errcode,
                           timeout_list, count_pending);

            if ((response_errcode == ER_WAVENRDY)
                    || (response_errcode == ER_PARAM))
            {
                if ((fpu.state == FPST_READY_FORWARD)
                        || (fpu.state == FPST_READY_BACKWARD))
                {
                    fpu.state = FPST_RESTING;
                }
                fpu.waveform_valid = false;
            }
            else if (response_status & STBT_ABORT_WAVE)
            {
                fpu.state = FPST_ABORTED;
                fpu.waveform_valid = false;
            }
            else if ((response_status & STBT_M1LIMIT) || (response_errcode == ER_M1LIMIT))
            {
                fpu.state = FPST_OBSTACLE_ERROR;
                fpu.waveform_valid = false;
                fpu.at_alpha_limit = true;
            }
            else if (response_errcode == ER_COLLIDE)
            {
                fpu.alpha_datum_switch_active = true;
                fpu.state = FPST_OBSTACLE_ERROR;
                fpu.waveform_valid = false;

            }

        }
        fpu.last_updated = cur_time;
        break;

    case CMSG_FINISHED_MOTION:
        // clear time-out flag
        remove_pending(fpu, fpu_id,  CCMD_EXECUTE_MOTION, response_errcode, timeout_list, count_pending);
        if (response_status & STBT_ABORT_WAVE)
        {
            fpu.state = FPST_ABORTED;
            fpu.movement_complete = false;
            fpu.waveform_valid = false;
        }
        else if ((response_status & STBT_M1LIMIT) || (response_errcode == ER_M1LIMIT))
        {
            fpu.at_alpha_limit = true;
            fpu.state = FPST_OBSTACLE_ERROR;
            fpu.movement_complete = false;
            fpu.waveform_valid = false;

        }
        else if (response_errcode == ER_COLLIDE)
        {
            fpu.state = FPST_OBSTACLE_ERROR;
            fpu.movement_complete = false;
            fpu.waveform_valid = false;

        }
        else if (response_errcode == ER_TIMING)
        {
            // we got a step timing error (meaning the interrupt
            // handler running on the FPUs microcontroller could not
            // compute the step frequency quick enough for the
            // configured microstepping level).
            //
            // FIXME: This should possibly generate an abortMotion
            // message for all FPUs, because other FPUs can crash into
            // the stopped one if they continue moving.
   	    printf("step timing error response received for FPU %i\n",
		 fpu_id);
	    fpu.state = FPST_ABORTED;
            fpu.movement_complete = false;
            fpu.waveform_valid = false;
	    fpu.step_timing_errcount++;

        }
        else if (response_errcode == 0)
        {
            // FIXME: Update step counter in protocol version 2
            // update_steps(fpu.alpha_steps, fpu.beta_steps, data);
            fpu.state = FPST_RESTING;
            fpu.movement_complete = true;

            // in protocol version 1, we do not know the last movement direction
            fpu.direction_alpha = DIRST_UNKNOWN;
            fpu.direction_beta = DIRST_UNKNOWN;
        }
        fpu.last_updated = cur_time;
        break;

    case CCMD_ABORT_MOTION    :
        // clear time-out flag
        if (response_errcode == 0)
        {
            // FIXME: Update step counter in protocol version 2
            //update_steps(fpu.alpha_steps, fpu.beta_steps, data);
            fpu.state = FPST_ABORTED;
            // remove executeMotion from pending commands
            switch(fpu.state)
            {
            case FPST_MOVING:
                remove_pending(fpu, fpu_id,  CCMD_EXECUTE_MOTION, response_errcode, timeout_list, count_pending);
                break;
            case FPST_DATUM_SEARCH:
                remove_pending(fpu, fpu_id,  CCMD_EXECUTE_MOTION, response_errcode, timeout_list, count_pending);
                break;
            default:
                /* the other commands are not movements */
                break;
            }
        }
        remove_pending(fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending);
        fpu.last_updated = cur_time;
        break;

    case CCMD_GET_STEPS_ALPHA :
        // clear time-out flag
        remove_pending(fpu, fpu_id,  cmd_id, response_errcode,timeout_list, count_pending);
        if (response_errcode == 0)
        {
            const uint16_t steps_coded = (data[5] << 8) | data[4];
            const int asteps = unfold_stepcount_alpha(steps_coded);
            fpu.alpha_steps = asteps;
        }
        fpu.last_updated = cur_time;
        if (fpu.state == FPST_UNKNOWN)
        {
            fpu.state = FPST_UNINITIALIZED;
        }
        break;

    case CCMD_GET_STEPS_BETA  :
        // clear time-out flag
        remove_pending(fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending);
        if (response_errcode == 0)
        {
            const uint16_t steps_coded = (data[5] << 8) |  data[4];
            const int bsteps = unfold_stepcount_beta(steps_coded);
            fpu.beta_steps = bsteps;
        }
        fpu.last_updated = cur_time;
        if (fpu.state == FPST_UNKNOWN)
        {
            fpu.state = FPST_UNINITIALIZED;
        }
        break;

    case CCMD_GET_ERROR_ALPHA  :
        // clear time-out flag
        remove_pending(fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending);
        if (response_errcode == 0)
        {
            const uint16_t steps_coded = (data[5] << 8) |  data[4];
            const int bsteps = unfold_steps_deviation(steps_coded);
            fpu.alpha_deviation = bsteps;
        }
        fpu.last_updated = cur_time;
        if (fpu.state == FPST_UNKNOWN)
        {
            fpu.state = FPST_UNINITIALIZED;
        }
        break;

    case CCMD_GET_ERROR_BETA  :
        // clear time-out flag
        remove_pending(fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending);
        if (response_errcode == 0)
        {
            const uint16_t steps_coded = (data[5] << 8) |  data[4];
            const int bsteps = unfold_steps_deviation(steps_coded);
            fpu.beta_deviation = bsteps;
        }
        fpu.last_updated = cur_time;
        if (fpu.state == FPST_UNKNOWN)
        {
            fpu.state = FPST_UNINITIALIZED;
        }
        break;

    case CCMD_PING_FPU        :
        // clear time-out flag
        remove_pending(fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending);
        fpu.last_updated = cur_time;
        if (fpu.state == FPST_UNKNOWN)
        {
            fpu.state = FPST_UNINITIALIZED;
        }

        if (response_errcode == 0)
        {
            const uint16_t asteps_coded = (data[5] << 8) |  data[4];
            const uint16_t bsteps_coded = (data[7] << 8) |  data[6];
            const int asteps = unfold_stepcount_alpha(asteps_coded);
            const int bsteps = unfold_stepcount_beta(bsteps_coded);

            fpu.alpha_steps = asteps;
            fpu.beta_steps = bsteps;
            fpu.ping_ok = true;
	    
#if (CAN_PROTOCOL_VERSION == 1)
	    // In protocol version 1, we (mis)use a ping after
	    // a repeatMotion to retrieve the FPU state
	    // and switch to READY_* again.
	    if ((fpu.waveform_valid)
		&& (fpu.waveform_ready)
		&& (fpu.state = FPST_RESTING))
	    {
	      if (fpu.waveform_reversed)
	      {
		fpu.state = FPST_READY_BACKWARD;
	      }
	      else
	      {
		fpu.state = FPST_READY_FORWARD;
	      }
	    }
#endif
        }
        else
        {
            fpu.ping_ok = false;
        }


        break;

    case CCMD_REPEAT_MOTION        :
        // clear time-out flag
        remove_pending(fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending);
        fpu.last_updated = cur_time;

        if ((response_errcode == 0)
                && fpu.waveform_valid
                && ( (fpu.state == FPST_RESTING) || (fpu.state == FPST_READY_BACKWARD)))
        {
            fpu.waveform_reversed = true;
            fpu.state = FPST_READY_FORWARD;
        }


        break;

    case CCMD_REVERSE_MOTION        :
        // clear time-out flag
        remove_pending(fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending);
        fpu.last_updated = cur_time;

        if ((response_errcode == 0)
                && fpu.waveform_valid
                && ( (fpu.state == FPST_RESTING) || (fpu.state == FPST_READY_FORWARD)))
        {
            fpu.waveform_reversed = true;
            fpu.state = FPST_READY_BACKWARD;
        }


        break;


    case CCMD_RESET_FPU       :
        // clear pending time-out
        remove_pending(fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending);
        if (response_errcode == 0)
        {
            initialize_fpu(fpu);
            update_status_flags(fpu, response_status);
        }
        fpu.last_updated = cur_time;

        // in protocol version 1, we do not know the last movement direction
        fpu.direction_alpha = DIRST_UNKNOWN;
        fpu.direction_beta = DIRST_UNKNOWN;
        break;

    case CCMD_FIND_DATUM :
        // we do not clear the pending flag, because
        // we wait for the datum search to finish
        //printf("confirmed: datum search for FPU %i \n", fpu_id);
        if (response_errcode == 0)
        {
            // datum search was successfully started
            fpu.state = FPST_DATUM_SEARCH;
            // in protocol version 1, we do not know the last movement direction
            fpu.direction_alpha = DIRST_UNKNOWN;
            fpu.direction_beta = DIRST_UNKNOWN;

            // As an edge case, it is possible that the command has
            // already been removed by a time-out handler. In that
            // case, re-add it as pending to avoid a stuck state.
            if (((fpu.pending_command_set >> CCMD_FIND_DATUM) & 1) == 0)
            {
#ifdef DEBUG
                printf("FPU #%i: WARNING: findDatum was removed from pending set, added again\n",
                       fpu_id);
#endif
                const timespec new_timeout = {40, 0};
                add_pending(fpu, fpu_id, CCMD_FIND_DATUM,
                            new_timeout,
                            timeout_list, count_pending);
            }
        }
        // we leave findDatum as pending command, because
        // we have to wait for the final response.
        fpu.last_updated = cur_time;
        break;

    case CMSG_FINISHED_DATUM :
        // clear time-out flag
        //printf("finished: datum search for FPU %i \n", fpu_id);
        remove_pending(fpu, fpu_id,  CCMD_FIND_DATUM, response_errcode, timeout_list, count_pending);

        if ((response_status & STBT_M1LIMIT) || (response_errcode == ER_M1LIMIT))
        {
            fpu.alpha_datum_switch_active = true;
            fpu.state = FPST_OBSTACLE_ERROR;
            fpu.waveform_valid = false;
            fpu.was_zeroed = false;
        }
        else if (response_errcode == ER_COLLIDE)
        {
            fpu.alpha_datum_switch_active = true;
            fpu.state = FPST_OBSTACLE_ERROR;
            fpu.waveform_valid = false;
            fpu.was_zeroed = false;                
        }
        else if (response_status & STBT_ABORT_WAVE)
        {
            fpu.state = FPST_ABORTED;
            fpu.waveform_valid = false;
        }
        else if (response_errcode != 0)
        {
            if (fpu.state == FPST_DATUM_SEARCH)
            {
                fpu.state = FPST_UNINITIALIZED;
                fpu.was_zeroed = false;
            }
        }
        else
        {
            // response_errcode was 0 and no bad status flags were set

            fpu.was_zeroed = true;
            fpu.alpha_steps = 0;
            fpu.beta_steps = 0;
            // in protocol version 1, we do not know the last movement direction
            fpu.direction_alpha = DIRST_UNKNOWN;
            fpu.direction_beta = DIRST_UNKNOWN;
            fpu.state = FPST_AT_DATUM;
        }

        fpu.last_updated = cur_time;
        break;

    case CMSG_WARN_COLLISION_BETA:
        printf("FPU # %i: beta collision message received \n", fpu_id);

        if (fpu.state == FPST_MOVING)
        {
            // clear time-out flag
            remove_pending(fpu, fpu_id,  CCMD_EXECUTE_MOTION, response_errcode, timeout_list, count_pending);

        }

        if (fpu.state == FPST_DATUM_SEARCH)
        {
            // clear time-out flag
            remove_pending(fpu, fpu_id,  CCMD_FIND_DATUM, response_errcode, timeout_list, count_pending);

        }

        fpu.state = FPST_OBSTACLE_ERROR;
        fpu.beta_collision = true;
        fpu.waveform_valid = false;
        fpu.was_zeroed = false;                
        // FIXME: Update step counter in protocol version 2
        //update_steps(fpu.alpha_steps, fpu.beta_steps, data);

        fpu.last_updated = cur_time;
        break;

    case CMSG_WARN_LIMIT_ALPHA:
        printf("FPU # %i: alpha limit breach message received \n", fpu_id);
        if (fpu.state == FPST_MOVING)
        {
            // clear time-out flag
            remove_pending(fpu, fpu_id,  CCMD_EXECUTE_MOTION, response_errcode, timeout_list, count_pending);

        }

        if (fpu.state == FPST_DATUM_SEARCH)
        {
            // clear time-out flag
            remove_pending(fpu, fpu_id,  CCMD_FIND_DATUM, response_errcode, timeout_list, count_pending);
        }

        fpu.state = FPST_OBSTACLE_ERROR;
        fpu.at_alpha_limit = true;
        fpu.waveform_valid = false;
        fpu.was_zeroed = false;                

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

        remove_pending(fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending);
        fpu.last_updated = cur_time;
        break;

    case CCMD_FREE_BETA_COLLISION:
        fpu.state = FPST_OBSTACLE_ERROR;
        fpu.waveform_valid = false;

        // FIXME: Update step counter in protocol version 2
        //update_steps(fpu.alpha_steps, fpu.beta_steps, data);

        remove_pending(fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending);
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
