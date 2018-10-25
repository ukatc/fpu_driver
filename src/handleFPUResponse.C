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
#include "ethercan/response_handlers/handle_AbortMotion_response.h"		     
#include "ethercan/response_handlers/handle_ConfigMotion_response.h"		     
#include "ethercan/response_handlers/handle_EnableBetaCollisionProtection_response.h"   
#include "ethercan/response_handlers/handle_ExecuteMotion_response.h"		     
#include "ethercan/response_handlers/handle_FindDatum_response.h"			     
#include "ethercan/response_handlers/handle_FinishedDatum_message.h"		     
#include "ethercan/response_handlers/handle_FinishedMotion_message.h"		     
#include "ethercan/response_handlers/handle_FreeBetaCollision_response.h"		     
#include "ethercan/response_handlers/handle_GetCounterDeviation_response.h"	     
#include "ethercan/response_handlers/handle_PingFPU_response.h"			     
#include "ethercan/response_handlers/handle_ReadRegister_response.h"		     
#include "ethercan/response_handlers/handle_ReadSerialNumber_response.h"		     
#include "ethercan/response_handlers/handle_RepeatMotion_response.h"		     
#include "ethercan/response_handlers/handle_ResetFPU_response.h"			     
#include "ethercan/response_handlers/handle_ReverseMotion_response.h"		     
#include "ethercan/response_handlers/handle_SetUStepLevel_response.h"		     
#include "ethercan/response_handlers/handle_WarnCANOverflow_warning.h"		     
#include "ethercan/response_handlers/handle_WarnCollisionBeta_warning.h"		     
#include "ethercan/response_handlers/handle_WarnLimitAlpha_warning.h"                   
#include "ethercan/response_handlers/handle_WriteSerialNumber_response.h"

#pragma GCC diagnostic ignored "-Wunused-parameter"


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
        handle_ConfigMotion_response(config, fpu_id, fpu, count_pending,  data, blen, timeout_list,
                                    cmd_id, response_status, response_errcode,
                                    cur_time);

        break;

    case CCMD_EXECUTE_MOTION  :
        handle_ExecuteMotion_response(config, fpu_id, fpu, count_pending,  data, blen, timeout_list,
                                     cmd_id, response_status, response_errcode,
                                     cur_time);
        break;

    case CMSG_FINISHED_MOTION:
        handle_FinishedMotion_message(config, fpu_id, fpu, count_pending,  data, blen, timeout_list,
                                      cmd_id, response_status, response_errcode,
                                      cur_time);

        break;

    case CCMD_ABORT_MOTION    :
        handle_AbortMotion_response(config, fpu_id, fpu, count_pending,  data, blen, timeout_list,
                                    cmd_id, response_status, response_errcode,
                                    cur_time);
        break;


    case CMSG_WARN_CANOVERFLOW    :
        handle_WarnCANOverflow_warning(config, fpu_id, fpu, count_pending,  data, blen, timeout_list,
                                       cmd_id, response_status, response_errcode,
                                       cur_time);

        break;


    case CCMD_GET_COUNTER_DEVIATION  :
        handle_GetCounterDeviation_response(config, fpu_id, fpu, count_pending,  data, blen, timeout_list,
                                            cmd_id, response_status, response_errcode,
                                            cur_time);
        break;

    case CCMD_PING_FPU        :
        handle_PingFPU_response(config, fpu_id, fpu, count_pending,  data, blen, timeout_list,
                                cmd_id, response_status, response_errcode,
                                cur_time);

        break;

    case CCMD_REPEAT_MOTION        :
        handle_RepeatMotion_response(config, fpu_id, fpu, count_pending,  data, blen, timeout_list,
                                    cmd_id, response_status, response_errcode,
                                    cur_time);

        break;

    case CCMD_REVERSE_MOTION        :
        handle_ReverseMotion_response(config, fpu_id, fpu, count_pending,  data, blen, timeout_list,
                                      cmd_id, response_status, response_errcode,
                                      cur_time);


        break;


    case CCMD_RESET_FPU       :
        handle_ResetFPU_response(config, fpu_id, fpu, count_pending,  data, blen, timeout_list,
                                 cmd_id, response_status, response_errcode,
                                 cur_time);

        break;

    case CCMD_FIND_DATUM :
        handle_FindDatum_response(config, fpu_id, fpu, count_pending,  data, blen, timeout_list,
                                  cmd_id, response_status, response_errcode,
                                  cur_time);
        break;

    case CMSG_FINISHED_DATUM :
        handle_FinishedDatum_message(config, fpu_id, fpu, count_pending,  data, blen, timeout_list,
                                     cmd_id, response_status, response_errcode,
                                     cur_time);
        break;

    case CMSG_WARN_COLLISION_BETA:
        handle_WarnCollisionBeta_warning(config, fpu_id, fpu, count_pending,  data, blen, timeout_list,
                                         cmd_id, response_status, response_errcode,
                                         cur_time);
        break;

    case CMSG_WARN_LIMIT_ALPHA:
        handle_WarnLimitAlpha_warning(config, fpu_id, fpu, count_pending,  data, blen, timeout_list,
                                      cmd_id, response_status, response_errcode,
                                      cur_time);
        break;


    case CCMD_ENABLE_BETA_COLLISION_PROTECTION:
        handle_EnableBetaCollisionProtection_response(config, fpu_id, fpu, count_pending,  data, blen, timeout_list,
                cmd_id, response_status, response_errcode,
                cur_time);
        break;

    case CCMD_FREE_BETA_COLLISION:
        handle_FreeBetaCollision_response(config, fpu_id, fpu, count_pending,  data, blen, timeout_list,
                                          cmd_id, response_status, response_errcode,
                                          cur_time);

        break;

    case CCMD_SET_USTEP_LEVEL:
        handle_SetUStepLevel_response(config, fpu_id, fpu, count_pending,  data, blen, timeout_list,
                                      cmd_id, response_status, response_errcode,
                                      cur_time);

        break;

    case CCMD_READ_REGISTER :
        handle_ReadRegister_response(config, fpu_id, fpu, count_pending,  data, blen, timeout_list,
                                     cmd_id, response_status, response_errcode,
                                     cur_time);

        break;


    case CCMD_WRITE_SERIAL_NUMBER:
        handle_WriteSerialNumber_response(config, fpu_id, fpu, count_pending,  data, blen, timeout_list,
                                          cmd_id, response_status, response_errcode,
                                          cur_time);

        break;

    case CCMD_READ_SERIAL_NUMBER  :
        handle_ReadSerialNumber_response(config, fpu_id, fpu, count_pending,  data, blen, timeout_list,
                                         cmd_id, response_status, response_errcode,
                                         cur_time);

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
