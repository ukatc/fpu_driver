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
#include "ethercan/handleFPUResponse.h"
#include "ethercan/time_utils.h"
#include "ethercan/FPUArray.h"


#ifdef DEBUG
#include <stdio.h>
#endif

namespace mpifps
{

namespace ethercanif
{

// logs error status in CAN response
void logErrorStatus(const EtherCANInterfaceConfig &config, int fpu_id, int err_code)
{

    const char * err_msg = "(no error)";
    switch (err_code)
    {
	
    case MCE_FPU_OK:
	err_msg = "no error";
        break;

    case MCE_WARN_COLLISION_DETECTED:
        err_msg = "FPU beta collision detected";
        break;

    case MCE_ERR_INVALID_COMMAND:
        err_msg = "invalid command received by motion controller";
        break;
	
    case MCE_ERR_WAVEFORM_NOT_READY:
        err_msg = "waveform not ready for execution";
        break;
	
    case MCE_WAVEFORM_TOO_BIG:
        err_msg = "too many waveform entries";
        break;
	
    case MCE_WARN_STEP_TIMING_ERROR:
        err_msg = "step timing error:microstepping value is too high for step frequency";
        break;
	
    case MCE_WARN_LIMIT_SWITCH_BREACH:
        err_msg = "alpha limit switch breach";
        break;
	
    case MCE_ERR_INVALID_PARAMETER:
        err_msg = "invalid parameter was rejected by motion controller";
        break;
	
    case MCE_WAVEFORM_SEQUENCE:
	err_msg = "transmitted waveform sequence not consistent in respect to use of first and last flags";
	break;
	
    case MCE_WAVEFORM_BADVALUE:
	err_msg = "the transmitted waveform value did not pass bounds checking";
	break;
	
    case MCE_ERR_DATUM_TIME_OUT:
	err_msg = "datum search exceeded hardware time or step limit";
	break;
	
    case MCE_NOTIFY_DATUM_ALPHA_ONLY:
	err_msg = "only the alpha arm was moved to datum";
	break;
	
    case MCE_NOTIFY_DATUM_BETA_ONLY:
	err_msg = "only the beta arm was moved to datum";
	break;
	
    case MCE_ERR_AUTO_DATUM_UNINITIALIZED:
	err_msg = "automatic datum operation was requested, but FPU is not initialized";
	break;
	
    case MCE_ERR_DATUM_ON_LIMIT_SWITCH:
	err_msg = "datum command was rejected because alpha arm is on limit switch";
	break;
		
    case MCE_ERR_CAN_OVERFLOW_HW:
	err_msg = "overflow in CAN hardware buffer";
	break;
	
    case MCE_ERR_CAN_OVERFLOW_SW:
	err_msg = "CAN overflow in motion controller firmware buffer";
	break;
	
    case MCE_NOTIFY_COMMAND_IGNORED:
	err_msg = "command was ignored by FPU motion controller";
	break;

    default:
        err_msg = "obsolete error code received";
        break;

    }

    LOG_RX(LOG_DEBUG, "%18.6f:FPU #%04i:error response msg = %s\n",
           get_realtime(),
           fpu_id, err_msg);

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
    const int lower_limit = -10000;
    const int upper_limit = lower_limit + (1 << 16) -1;
    if (val > upper_limit)
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
    const int upper_limit = (1 << 15) -1;
    if (val > upper_limit)
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

void handleFPUResponse(const EtherCANInterfaceConfig& config,
                       int fpu_id, t_fpu_state& fpu,
                       const t_response_buf& data,
                       const int blen, TimeOutList& timeout_list,
                       int &count_pending)
{
    E_CAN_COMMAND cmd_id = static_cast<E_CAN_COMMAND>(data[1]);
    uint8_t response_status = data[2];
    update_status_flags(fpu, response_status);

    E_MOC_ERRCODE response_errcode = data[3] ? static_cast<E_MOC_ERRCODE>(data[4]) : MCE_FPU_OK;
    timespec cur_time;

    assert(blen == 8);

    get_monotonic_time(cur_time);

    switch (cmd_id)
    {
    case CCMD_CONFIG_MOTION   :

        LOG_RX(LOG_TRACE_CAN_MESSAGES, "%18.6f : RX : handle_ConfigMotion:"
               " fpu #%u, segment %u: status=%u, error=%u\n",
               get_realtime(),
               fpu_id, fpu.num_waveform_segments,
               response_status, response_errcode);

        // clear time-out flag
        remove_pending(config, fpu, fpu_id, cmd_id, response_errcode, timeout_list, count_pending);
        if (response_errcode != 0)
        {
            logErrorStatus(config, fpu_id, response_errcode);
            // if the FPU was in loading state, it is switched to RESTING,
            // otherwise unchanged.

            // FIXME: decrease log level in production system to keep responsivity at maximum
            LOG_RX(LOG_ERROR, "%18.6f : RX : "
                   "configMotion command for FPU %i failed with error code %i\n",
                   get_realtime(),
                   fpu_id,
                   response_errcode);

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
            if ((fpu.state != FPST_OBSTACLE_ERROR) && (fpu.state != FPST_ABORTED))
            {
                fpu.state = FPST_MOVING;
            }
            fpu.movement_complete = false;;
            // status byte should show RUNNING_WAVE, too

            // in protocol version 1, we do not know the last movement direction
            fpu.direction_alpha = DIRST_UNKNOWN;
            fpu.direction_beta = DIRST_UNKNOWN;
            fpu.ping_ok = false;

            // As an edge case, if the confirmation arrives extremely
            // large, then it is possible that the command has already
            // been removed by a time-out handler. In that case,
            // re-add it as pending to avoid a stuck state.
            if (((fpu.pending_command_set >> CCMD_EXECUTE_MOTION) & 1) == 0)
            {
                LOG_RX(LOG_DEBUG, "%18.6f : RX : "
                       "FPU #%i: WARNING: executeMotion was removed from pending set (%0x), added again\n",
                       get_realtime(),
                       fpu_id, fpu.pending_command_set);

                const timespec new_timeout = {40, 0};
                add_pending(fpu, fpu_id, CCMD_EXECUTE_MOTION,
                            new_timeout,
                            timeout_list, count_pending);
            }
        }
        else
        {
            // clear timeout status
            remove_pending(config, fpu, fpu_id,  CCMD_EXECUTE_MOTION, response_errcode,
                           timeout_list, count_pending);

            // FIXME: decrease log level in production system to keep responsivity at maximum
            LOG_RX(LOG_ERROR, "%18.6f : RX : "
                   "FPU # %i: executeMotion command got error response code #%i,"
                   " removed from pending list.\n",
                   get_realtime(),
                   fpu_id,
                   response_errcode);

            if ((response_errcode == MCE_ERR_WAVEFORM_NOT_READY)
                    || (response_errcode == MCE_ERR_INVALID_PARAMETER))
            {
                if ((fpu.state == FPST_READY_FORWARD)
                        || (fpu.state == FPST_READY_REVERSE))
                {
                    fpu.state = FPST_RESTING;
                }
                fpu.waveform_valid = false;

                // This message is set to a low log level because it is sent by any extra FPUs
                // which have not been configured
                LOG_CONSOLE(LOG_DEBUG, "%18.6f : RX : "
                            "FPU # %i: executeMotion command got error response 'ER_WAVENREDY' / 'MCE_ERR_INVALID_PARAMETER'"
                            " command skipped.\n",
                            get_realtime(),
                            fpu_id);
            }
            else if ((response_status & STBT_M1LIMIT) || (response_errcode == MCE_WARN_LIMIT_SWITCH_BREACH))
            {
                fpu.state = FPST_OBSTACLE_ERROR;
                fpu.waveform_valid = false;
                fpu.at_alpha_limit = true;
                fpu.ping_ok = false;

                LOG_CONSOLE(LOG_ERROR, "%18.6f : RX : "
                            "FPU # %i: executeMotion command got error status 'MCE_WARN_LIMIT_SWITCH_BREACH'/'STBT_M1LIMIT'"
                            " command cancelled.\n",
                            get_realtime(),
                            fpu_id);

            }
            else if (response_errcode == MCE_WARN_COLLISION_DETECTED)
            {
                fpu.alpha_datum_switch_active = true;
                fpu.state = FPST_OBSTACLE_ERROR;
                fpu.waveform_valid = false;

                LOG_CONSOLE(LOG_ERROR, "%18.6f : RX : "
                            "FPU # %i: executeMotion command got error response code 'MCE_WARN_COLLISION_DETECTED'"
                            " command cancelled.\n",
                            get_realtime(),
                            fpu_id);
            }
            else if (response_status & STBT_ABORT_WAVE)
            {
                if (fpu.state != FPST_OBSTACLE_ERROR)
                {
                    fpu.state = FPST_ABORTED;
                }
                fpu.waveform_valid = false;
                fpu.ping_ok = false;

                LOG_CONSOLE(LOG_ERROR, "%18.6f : RX : "
                            "FPU # %i: executeMotion command got error status 'STBT_ABORT_WAVE'"
                            " command cancelled.\n",
                            get_realtime(),
                            fpu_id);
            }

        }
        fpu.last_updated = cur_time;
        break;

    case CMSG_FINISHED_MOTION:
        // clear time-out flag
        remove_pending(config, fpu, fpu_id,  CCMD_EXECUTE_MOTION, response_errcode, timeout_list, count_pending);
        if ((response_errcode == MCE_WARN_COLLISION_DETECTED) || fpu.beta_collision)
        {
            fpu.state = FPST_OBSTACLE_ERROR;
            fpu.movement_complete = false;
            fpu.waveform_valid = false;
            fpu.beta_collision = true;
            fpu.waveform_valid = false;
            fpu.alpha_was_zeroed = false;
            fpu.beta_was_zeroed = false;
            fpu.ping_ok = false;

            // FIXME: decrease log level in production system to keep responsivity at maximum
            LOG_RX(LOG_ERROR, "%18.6f : RX : "
                   "while waiting for finishedMotion: "
                   "collision detected message received for FPU %i\n",
                   get_realtime(),
                   fpu_id);

            LOG_CONSOLE(LOG_VERBOSE, "%18.6f : RX : "
                        "FPU # %i: executeMotion command finished error status "
                        "'MCE_WARN_COLLISION_DETECTED (beta arm collision)'"
                        " movement aborted.\n",
                        get_realtime(),
                        fpu_id);
        }
        else if ((response_status & STBT_M1LIMIT) || (response_errcode == MCE_WARN_LIMIT_SWITCH_BREACH) || fpu.at_alpha_limit)
        {
            fpu.at_alpha_limit = true;
            fpu.state = FPST_OBSTACLE_ERROR;
            fpu.movement_complete = false;
            fpu.waveform_valid = false;
            fpu.alpha_was_zeroed = false;
            fpu.beta_was_zeroed = false;
            fpu.alpha_datum_switch_active = true;
            fpu.ping_ok = false;

            // FIXME: decrease log level in production system to keep responsivity at maximum
            LOG_RX(LOG_ERROR, "%18.6f : RX : "
                   "while waiting for finishedMotion: "
                   "limit switch breach message received for FPU %i\n",
                   get_realtime(),
                   fpu_id);

            LOG_CONSOLE(LOG_VERBOSE, "%18.6f : RX : "
                        "FPU # %i: executeMotion command finished error status "
                        "'MCE_WARN_LIMIT_SWITCH_BREACH' (alpha limit switch breach)"
                        " movement aborted.\n",
                        get_realtime(),
                        fpu_id);

        }
        else if (response_status & STBT_ABORT_WAVE)
        {
            if (fpu.state != FPST_OBSTACLE_ERROR)
            {
                fpu.state = FPST_ABORTED;
            }
            fpu.movement_complete = false;
            fpu.waveform_valid = false;
            fpu.ping_ok = false;

            LOG_RX(LOG_ERROR, "%18.6f : RX : "
                   "FPU # %i: executeMotion command finished error status 'FPST_ABORTED'"
                   " movement aborted.\n",
                   get_realtime(),
                   fpu_id);

            LOG_CONSOLE(LOG_VERBOSE, "%18.6f : RX : "
                        "FPU # %i: executeMotion command finished error status 'FPST_ABORTED'"
                        " movement aborted.\n",
                        get_realtime(),
                        fpu_id);
        }
        else if (response_errcode == MCE_WARN_STEP_TIMING_ERROR)
        {
            // we got a step timing error (meaning the interrupt
            // handler running on the FPUs microcontroller could not
            // compute the step frequency quick enough for the
            // configured microstepping level).
            //
            // FIXME: This should possibly generate an abortMotion
            // message for all FPUs, because other FPUs can crash into
            // the stopped one if they continue moving.

            LOG_RX(LOG_ERROR, "%18.6f : RX : "
                   "while waiting for finishedMotion: "
                   "step timing error response received for FPU %i\n",
                   get_realtime(),
                   fpu_id);

            LOG_CONSOLE(LOG_VERBOSE, "%18.6f : RX : "
                        "FPU # %i: executeMotion command finished error status "
                        "'MCE_WARN_STEP_TIMING_ERROR (step timing error / firmware error)'"
                        " movement aborted.\n",
                        get_realtime(),
                        fpu_id);

            if (fpu.state != FPST_OBSTACLE_ERROR)
            {
                fpu.state = FPST_ABORTED;
            }
            fpu.movement_complete = false;
            fpu.waveform_valid = false;
            fpu.step_timing_errcount++;
            fpu.ping_ok = false;

        }
        else if (response_errcode == 0)
        {
            // FIXME: Update step counter in protocol version 2
            // update_steps(fpu.alpha_steps, fpu.beta_steps, data);
            if ( (fpu.state != FPST_OBSTACLE_ERROR) && (fpu.state != FPST_ABORTED))
            {
                fpu.state = FPST_RESTING;
                fpu.movement_complete = true;
                fpu.ping_ok = false;
            }

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
            if (fpu.state != FPST_OBSTACLE_ERROR)
            {
                fpu.state = FPST_ABORTED;
            }
            // remove executeMotion from pending commands
            switch(fpu.state)
            {
            case FPST_MOVING:
                remove_pending(config, fpu, fpu_id,  CCMD_EXECUTE_MOTION, response_errcode, timeout_list, count_pending);
                break;
            case FPST_DATUM_SEARCH:
                remove_pending(config, fpu, fpu_id,  CCMD_FIND_DATUM, response_errcode, timeout_list, count_pending);
                break;
            default:
                /* the other commands are not movements */
                break;
            }
        }
        remove_pending(config, fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending);
        fpu.last_updated = cur_time;
        fpu.ping_ok = false;

	// this is set to a low logging level because any moving FPU
	// will send this message
	LOG_RX(LOG_DEBUG, "%18.6f : RX : "
	       "abortMotion message received for FPU %i\n",
	       get_realtime(),
	       fpu_id);


        break;


    case CMSG_WARN_CANOVERFLOW    :
        // clear time-out flag

        // FIXME: Update step counter in protocol version 2
        //update_steps(fpu.alpha_steps, fpu.beta_steps, data);
        // remove executeMotion from pending commands
        switch(fpu.state)
        {
        case FPST_MOVING:
            remove_pending(config, fpu, fpu_id,  CCMD_EXECUTE_MOTION, response_errcode, timeout_list, count_pending);
            break;
        case FPST_DATUM_SEARCH:
            remove_pending(config, fpu, fpu_id,  CCMD_FIND_DATUM, response_errcode, timeout_list, count_pending);
            break;
        default:
            /* the other commands are not movements */
            break;
        }

        // the most likely situation in which an overflow response occurs is
        // when uploading new waveforms to the FPUs
        if ( ((fpu.pending_command_set >> CCMD_CONFIG_MOTION) & 1) == 1)
        {

            remove_pending(config, fpu, fpu_id, CCMD_CONFIG_MOTION, response_errcode, timeout_list, count_pending);

            if (fpu.state == FPST_LOADING)
            {
                fpu.state = FPST_RESTING;
            }

        }

        fpu.last_updated = cur_time;
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

        break;


    case CCMD_GET_COUNTER_DEVIATION  :
        // clear time-out flag
        remove_pending(config, fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending);
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

    case CCMD_PING_FPU        :
        // clear time-out flag
        remove_pending(config, fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending);
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

#if 0
            // In protocol version 1, we (mis)use a ping after
            // a repeatMotion to retrieve the FPU state
            // and switch to READY_* again.
            if ((fpu.waveform_valid)
                    && (fpu.waveform_ready)
                    && (fpu.state = FPST_RESTING))
            {
                if (fpu.waveform_reversed)
                {
                    fpu.state = FPST_READY_REVERSE;
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

            // FIXME: decrease log level in production system to keep responsivity at maximum
            LOG_RX(LOG_ERROR, "%18.6f : RX : "
                   "pingFPU command failed for FPU %i\n",
                   get_realtime(),
                   fpu_id);
        }


        break;

    case CCMD_REPEAT_MOTION        :
        // clear time-out flag
        remove_pending(config, fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending);
        fpu.last_updated = cur_time;

        if ((response_errcode == 0)
                && fpu.waveform_valid
                && ( (fpu.state == FPST_RESTING) || (fpu.state == FPST_READY_REVERSE)))
        {
            fpu.waveform_reversed = true;
            fpu.state = FPST_READY_FORWARD;
        }


        break;

    case CCMD_REVERSE_MOTION        :
        // clear time-out flag
        remove_pending(config, fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending);
        fpu.last_updated = cur_time;

        if ((response_errcode == 0)
                && fpu.waveform_valid
                && ( (fpu.state == FPST_RESTING) || (fpu.state == FPST_READY_FORWARD)))
        {
            fpu.waveform_reversed = true;
            fpu.state = FPST_READY_REVERSE;
        }


        break;


    case CCMD_RESET_FPU       :
        // clear pending time-out for reset command
        remove_pending(config, fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending);
        if (response_errcode == 0)
        {
            initialize_fpu(fpu);
            fpu.state = FPST_UNINITIALIZED; // instead of unknown - we known the step count is zero
            update_status_flags(fpu, response_status);

            if (fpu.pending_command_set != 0)
            {
                // remove *all* other pending commands
                for(int cmd_code=0; cmd_code < NUM_CAN_COMMANDS; cmd_code++)
                {
                    if (((fpu.pending_command_set >> cmd_code) & 1) == 1)
                    {
                        const E_CAN_COMMAND can_cmd = static_cast<E_CAN_COMMAND>(cmd_code);
                        remove_pending(config, fpu, fpu_id,  can_cmd, response_errcode,
                                       timeout_list, count_pending);
                    }

                }
            }
        }
        fpu.last_updated = cur_time;
        fpu.ping_ok = true; // we known the current step counter

        // in protocol version 1, we do not know the last movement direction
        fpu.direction_alpha = DIRST_UNKNOWN;
        fpu.direction_beta = DIRST_UNKNOWN;
        break;

    case CCMD_FIND_DATUM :
        // we do not clear the pending flag, because
        // we wait for the datum search to finish
        if (response_errcode == 0)
        {
            // datum search was successfully started
            fpu.state = FPST_DATUM_SEARCH;
            // in protocol version 1, we do not know the last movement direction
            fpu.direction_alpha = DIRST_UNKNOWN;
            fpu.direction_beta = DIRST_UNKNOWN;
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
                            timeout_list, count_pending);
            }
        }
        else if (response_errcode == MCE_ERR_DATUM_ON_LIMIT_SWITCH)
        {
            remove_pending(config, fpu, fpu_id,  CCMD_FIND_DATUM, response_errcode, timeout_list, count_pending);

            fpu.state = FPST_UNINITIALIZED;
            fpu.alpha_was_zeroed = false;
            fpu.beta_was_zeroed = false;
            fpu.ping_ok = false;

            LOG_RX(LOG_ERROR, "%18.6f : RX : "
                   "findDatum request rejected for FPU %i, because alpha limit switch active\n",
                   get_realtime(),
                   fpu_id);
        }
        else if (response_errcode == MCE_ERR_AUTO_DATUM_UNINITIALIZED)
        {
            remove_pending(config, fpu, fpu_id,  CCMD_FIND_DATUM, response_errcode, timeout_list, count_pending);

            fpu.state = FPST_UNINITIALIZED;
            fpu.alpha_was_zeroed = false;
            fpu.beta_was_zeroed = false;

            LOG_RX(LOG_ERROR, "%18.6f : RX : "
                   "error:"
                   "FPU %i was not initialised, automatic datum search rejected\n",
                   get_realtime(),
                   fpu_id);

        }

        // we leave findDatum as pending command, because
        // we have to wait for the final response.
        fpu.last_updated = cur_time;
        break;

    case CMSG_FINISHED_DATUM :
        // clear time-out flag
        //printf("finished: datum search for FPU %i \n", fpu_id);
        remove_pending(config, fpu, fpu_id,  CCMD_FIND_DATUM, response_errcode, timeout_list, count_pending);

        if ((response_status & STBT_M1LIMIT) || (response_errcode == MCE_WARN_LIMIT_SWITCH_BREACH))
        {
            fpu.alpha_datum_switch_active = true;
            fpu.at_alpha_limit = true;
            fpu.state = FPST_OBSTACLE_ERROR;
            fpu.waveform_valid = false;
            fpu.alpha_was_zeroed = false;
            fpu.beta_was_zeroed = false;
            fpu.ping_ok = false;

            // FIXME: decrease log level in production system to keep responsivity at maximum
            LOG_RX(LOG_ERROR, "%18.6f : RX : "
                   "while waiting for end of datum command:"
                   "limit switch breach message received for FPU %i\n",
                   get_realtime(),
                   fpu_id);
        }
        else if ((response_errcode == MCE_WARN_COLLISION_DETECTED) || fpu.beta_collision)
        {
            fpu.beta_collision = true;
            fpu.state = FPST_OBSTACLE_ERROR;
            fpu.waveform_valid = false;
            fpu.alpha_was_zeroed = false;
            fpu.beta_was_zeroed = false;
            fpu.ping_ok = false;

            // FIXME: decrease log level in production system to keep responsivity at maximum
            LOG_RX(LOG_ERROR, "%18.6f : RX : "
                   "while waiting for end of datum command:"
                   "collision detection message received for FPU %i\n",
                   get_realtime(),
                   fpu_id);
        }
        else if (response_errcode == MCE_ERR_DATUM_TIME_OUT)
        {
            // The datum operation was timed-out by the firmware.  The
            // only way this can regularly happen for the alpha arm is
            // if the motor is not moving, or the alpha switch is
            // broken. In the latter case, the alpha gearbox is
            // possibly already destroyed.
            fpu.state = FPST_ABORTED;
            fpu.waveform_valid = false;
            fpu.alpha_was_zeroed = false;
            fpu.beta_was_zeroed = false;
            fpu.ping_ok = false;

            // FIXME: decrease log level in production system to keep responsivity at maximum
            LOG_RX(LOG_ERROR, "%18.6f : RX : "
                   "while waiting for finishing datum command:"
                   "hardware datum time-out message received for FPU %i\n",
                   get_realtime(),
                   fpu_id);

            LOG_CONSOLE(LOG_ERROR, "%18.6f : RX : "
                        "while waiting for finishing datum command:"
                        "hardware datum time-out message received for FPU %i\n"
                        "\a\a\aWARNING: HARDWARE DAMAGE LIKELY\n",
                        get_realtime(),
                        fpu_id);
        }
        else if (response_status & STBT_ABORT_WAVE)
        {
            if (fpu.state != FPST_OBSTACLE_ERROR)
            {
                fpu.state = FPST_ABORTED;
            }
            fpu.waveform_valid = false;
            fpu.ping_ok = false;

            LOG_RX(LOG_DEBUG, "%18.6f : RX : "
                   "while waiting for end of datum command:"
                   "movement abortion message received for FPU %i\n",
                   get_realtime(),
                   fpu_id);
        }
        else if (response_errcode == MCE_ERR_DATUM_ON_LIMIT_SWITCH)
        {
            if (fpu.state == FPST_DATUM_SEARCH)
            {
                fpu.state = FPST_UNINITIALIZED;
                fpu.alpha_was_zeroed = false;
                fpu.beta_was_zeroed = false;
                fpu.ping_ok = false;
            }
            LOG_RX(LOG_ERROR, "%18.6f : RX : "
                   "datum request rejected for FPU %i, because alpha limit switch active\n",
                   get_realtime(),
                   fpu_id);
        }
        else if (response_errcode != 0)
        {
            if (fpu.state == FPST_DATUM_SEARCH)
            {
                fpu.state = FPST_UNINITIALIZED;
                fpu.alpha_was_zeroed = false;
                fpu.beta_was_zeroed = false;
                fpu.ping_ok = false;
            }
        }
        else
        {
            // response_errcode was 0 and no bad status flags were set

            uint8_t exclusion_flags = data[5];
            if ((exclusion_flags & DATUM_SKIP_ALPHA) == 0)
            {
                fpu.alpha_was_zeroed = true;
                fpu.alpha_steps = 0;
            }
            if ((exclusion_flags & DATUM_SKIP_BETA) == 0)
            {
                fpu.beta_was_zeroed = true;
                fpu.beta_steps = 0;
            }
            if (fpu.beta_was_zeroed && fpu.alpha_was_zeroed)
            {
                fpu.state = FPST_AT_DATUM;
                fpu.ping_ok = true;
            }
            else
            {
                // if only one arm was zeroed, the state still counts as
                // uninitialized (zeroing only one arm is used only
                // for testing and engineering, not for instrument operation).
                fpu.state = FPST_UNINITIALIZED;
            }
            // in protocol version 1, we do not know the last movement direction
            fpu.direction_alpha = DIRST_UNKNOWN;
            fpu.direction_beta = DIRST_UNKNOWN;
        }

        fpu.last_updated = cur_time;
        break;

    case CMSG_WARN_COLLISION_BETA:
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
            remove_pending(config, fpu, fpu_id,  CCMD_EXECUTE_MOTION, response_errcode, timeout_list, count_pending);

        }

        if (fpu.state == FPST_DATUM_SEARCH)
        {
            // clear time-out flag
            remove_pending(config, fpu, fpu_id,  CCMD_FIND_DATUM, response_errcode, timeout_list, count_pending);

        }

        fpu.state = FPST_OBSTACLE_ERROR;
        fpu.beta_collision = true;
        fpu.waveform_valid = false;
        fpu.alpha_was_zeroed = false;
        fpu.beta_was_zeroed = false;
        fpu.ping_ok = false;

        // FIXME: Update step counter in protocol version 2
        //update_steps(fpu.alpha_steps, fpu.beta_steps, data);

        fpu.last_updated = cur_time;
        break;

    case CMSG_WARN_LIMIT_ALPHA:
        if (fpu.state == FPST_MOVING)
        {
            // clear time-out flag
            remove_pending(config, fpu, fpu_id,  CCMD_EXECUTE_MOTION, response_errcode, timeout_list, count_pending);

        }

        if (fpu.state == FPST_DATUM_SEARCH)
        {
            // clear time-out flag
            remove_pending(config, fpu, fpu_id,  CCMD_FIND_DATUM, response_errcode, timeout_list, count_pending);
        }

        LOG_RX(LOG_ERROR, "%18.6f : RX : "
               "limit switch breach message received for FPU %i\n",
               get_realtime(),
               fpu_id);

        LOG_CONSOLE(LOG_ERROR, "%18.6f : RX : "
                    "FPU # %i: alpha arm limit switch breach message received.\n",
                    get_realtime(),
                    fpu_id);

        fpu.state = FPST_OBSTACLE_ERROR;
        fpu.at_alpha_limit = true;
        fpu.waveform_valid = false;
        fpu.alpha_was_zeroed = false;
        fpu.beta_was_zeroed = false;
        fpu.ping_ok = false;

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

        remove_pending(config, fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending);
        fpu.last_updated = cur_time;
        break;

    case CCMD_FREE_BETA_COLLISION:
        fpu.state = FPST_OBSTACLE_ERROR;
        fpu.waveform_valid = false;

        // FIXME: Update step counter in protocol version 2
        //update_steps(fpu.alpha_steps, fpu.beta_steps, data);

        remove_pending(config, fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending);
        fpu.last_updated = cur_time;
        fpu.ping_ok = false;
        break;

    case CCMD_SET_USTEP_LEVEL:
        fpu.ping_ok = true;

        remove_pending(config, fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending);
        fpu.last_updated = cur_time;
        break;

    case CCMD_READ_REGISTER :
        // clear time-out flag
        remove_pending(config, fpu, fpu_id,  cmd_id, response_errcode,timeout_list, count_pending);
        if (response_errcode == 0)
        {
            fpu.register_value = data[4];
            // for protcol version 2, also read the echoed memory address
        }
        fpu.last_updated = cur_time;
        if (fpu.state == FPST_UNKNOWN)
        {
            fpu.state = FPST_UNINITIALIZED;
        }
        break;


    case CCMD_WRITE_SERIAL_NUMBER:

        remove_pending(config, fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending);
        fpu.last_updated = cur_time;
        break;

    case CCMD_READ_SERIAL_NUMBER  :
        // clear time-out flag
        remove_pending(config, fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending);
        // ignore the error code, it is not set in protocol V1, for space reasons
        //if (response_errcode == 0)
        {
            memset(fpu.serial_number, 0, sizeof(fpu.serial_number));
            static_assert(DIGITS_SERIAL_NUMBER < sizeof(fpu.serial_number), "buffer overflow");
            strncpy(fpu.serial_number, (char*) (data + 3), DIGITS_SERIAL_NUMBER);

            LOG_RX(LOG_VERBOSE, "%18.6f : RX : "
                   "Serial number for FPU %i is reported as %s\n",
                   get_realtime(),
                   fpu_id,
                   fpu.serial_number);
        }
        fpu.last_updated = cur_time;
        break;

    case CCMD_NO_COMMAND      :
    default:
        // invalid command, ignore
        LOG_RX(LOG_ERROR, "%18.6f : RX : "
               "invalid message (id # %i) received for FPU %i\n",
               get_realtime(),
               cmd_id,
               fpu_id);
        break;

    }

}

}

}
