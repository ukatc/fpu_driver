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
// lines below alphabetically sorted
#include "ethercan/response_handlers/handle_AbortMotion_response.h"
#include "ethercan/response_handlers/handle_CheckIntegrity_response.h"
#include "ethercan/response_handlers/handle_ConfigMotion_response.h"
#include "ethercan/response_handlers/handle_EnableAlphaLimitProtection_response.h"
#include "ethercan/response_handlers/handle_EnableBetaCollisionProtection_response.h"
#include "ethercan/response_handlers/handle_EnableMove_response.h"
#include "ethercan/response_handlers/handle_ExecuteMotion_response.h"
#include "ethercan/response_handlers/handle_FindDatum_response.h"
#include "ethercan/response_handlers/handle_FinishedDatum_message.h"
#include "ethercan/response_handlers/handle_FinishedMotion_message.h"
#include "ethercan/response_handlers/handle_FreeAlphaLimitBreach_response.h"
#include "ethercan/response_handlers/handle_FreeBetaCollision_response.h"
#include "ethercan/response_handlers/handle_GetFirmwareVersion_response.h"
#include "ethercan/response_handlers/handle_LockUnit_response.h"
#include "ethercan/response_handlers/handle_PingFPU_response.h"
#include "ethercan/response_handlers/handle_ReadRegister_response.h"
#include "ethercan/response_handlers/handle_ReadSerialNumber_response.h"
#include "ethercan/response_handlers/handle_RepeatMotion_response.h"
#include "ethercan/response_handlers/handle_ResetFPU_response.h"
#include "ethercan/response_handlers/handle_ResetStepCounter_response.h"
#include "ethercan/response_handlers/handle_ReverseMotion_response.h"
#include "ethercan/response_handlers/handle_SetStepsPerSegment_response.h"
#include "ethercan/response_handlers/handle_SetTicksPerSegment_response.h"
#include "ethercan/response_handlers/handle_SetUStepLevel_response.h"
#include "ethercan/response_handlers/handle_UnlockUnit_response.h"
#include "ethercan/response_handlers/handle_WarnCANOverflow_warning.h"
#include "ethercan/response_handlers/handle_WarnCollisionBeta_warning.h"
#include "ethercan/response_handlers/handle_WarnLimitAlpha_warning.h"
#include "ethercan/response_handlers/handle_WarnLimitAlpha_warning.h"
#include "ethercan/response_handlers/handle_WriteSerialNumber_response.h"



#ifdef DEBUG
#include <stdio.h>
#endif

namespace mpifps
{

namespace ethercanif
{


void handleFPUResponse(const EtherCANInterfaceConfig& config,
                       int fpu_id, t_fpu_state& fpu,
                       const t_response_buf& data,
                       const int blen, TimeOutList& timeout_list,
                       int &count_pending)
{
    void (*handler) (const EtherCANInterfaceConfig&,
                     const int,
                     t_fpu_state&,
                     int &,
                     const t_response_buf&,
                     const int,
                     TimeOutList&t,
                     const E_CAN_COMMAND,
                     const uint8_t) = nullptr;

    E_CAN_COMMAND cmd_id = static_cast<E_CAN_COMMAND>(data[1] & COMMAND_CODE_MASK);
    int required_length = 8;

    switch (cmd_id)
    {
    case CCMD_ABORT_MOTION                     :
        handler = &handle_AbortMotion_response;
        break;
    case CCMD_CONFIG_MOTION                    :
        handler = &handle_ConfigMotion_response;
        required_length = 5;
        break;
    case CCMD_FREE_BETA_COLLISION              :
        handler = &handle_FreeBetaCollision_response;
        break;
    case CCMD_ENABLE_BETA_COLLISION_PROTECTION :
        handler = &handle_EnableBetaCollisionProtection_response;
        break;
    case CCMD_EXECUTE_MOTION                   :
        handler = &handle_ExecuteMotion_response;
        break;
    case CCMD_FIND_DATUM                       :
        handler = &handle_FindDatum_response;
        break;
    case CCMD_PING_FPU                         :
        handler = &handle_PingFPU_response;
        break;
    case CCMD_READ_REGISTER                    :
        handler = &handle_ReadRegister_response;
        required_length = 7;
        break;
    case CCMD_READ_SERIAL_NUMBER               :
        handler = &handle_ReadSerialNumber_response;
        break;
    case CCMD_REPEAT_MOTION                    :
        handler = &handle_RepeatMotion_response;
        break;
    case CCMD_RESET_FPU                        :
        handler = &handle_ResetFPU_response;
        break;
    case CCMD_REVERSE_MOTION                   :
        handler = &handle_ReverseMotion_response;
        break;
    case CCMD_SET_USTEP_LEVEL                  :
        handler = &handle_SetUStepLevel_response;
        break;
    case CCMD_WRITE_SERIAL_NUMBER              :
        handler = &handle_WriteSerialNumber_response;
        break;
    case CMSG_FINISHED_DATUM                   :
        handler = &handle_FinishedDatum_message;
        break;
    case CMSG_FINISHED_MOTION                  :
        handler = &handle_FinishedMotion_message;
        break;
    case CMSG_WARN_CANOVERFLOW                 :
        handler = &handle_WarnCANOverflow_warning;
        break;
    case CMSG_WARN_COLLISION_BETA              :
        handler = &handle_WarnCollisionBeta_warning;
        break;
    case CMSG_WARN_LIMIT_ALPHA                 :
        handler = &handle_WarnLimitAlpha_warning;
        break;
    case CCMD_LOCK_UNIT                        :
        handler = &handle_LockUnit_response;
        break;
    case CCMD_UNLOCK_UNIT                      :
        handler = &handle_UnlockUnit_response;
        break;
    case CCMD_RESET_STEPCOUNTER                :
        handler = &handle_ResetStepCounter_response;
        break;
    case CCMD_GET_FIRMWARE_VERSION             :
        handler = &handle_GetFirmwareVersion_response;
        break;
    case CCMD_CHECK_INTEGRITY                  :
        handler = &handle_CheckIntegrity_response;
        break;
    case CCMD_FREE_ALPHA_LIMIT_BREACH          :
        handler = &handle_FreeAlphaLimitBreach_response;
        break;
    case CCMD_ENABLE_ALPHA_LIMIT_PROTECTION    :
        handler = &handle_EnableAlphaLimitProtection_response;
        break;
    case CCMD_SET_TICKS_PER_SEGMENT            :
        handler = &handle_SetTicksPerSegment_response;
        break;
    case CCMD_SET_STEPS_PER_SEGMENT            :
        handler = &handle_SetStepsPerSegment_response;
        break;
    case CCMD_ENABLE_MOVE                      :
        handler = &handle_EnableMove_response;
        break;


    case CCMD_NO_COMMAND      :
    default:
        // invalid command, ignore
        handler = nullptr;
        LOG_RX(LOG_ERROR, "%18.6f : RX : "
               "invalid message (id # %i) received for FPU %i\n",
               get_realtime(),
               cmd_id,
               fpu_id);
        return;

    }
    if (blen != required_length)
    {
        LOG_RX(LOG_ERROR, "%18.6f : RX : "
               "invalid message length (%i, should be %i) for command id # %i) received for FPU %i\n",
               get_realtime(),
               blen,
               required_length,
               cmd_id,
               fpu_id);
        return;
    }

    if (handler != nullptr)
    {
        uint8_t sequence_number = data[0];

        (*handler)(config, fpu_id, fpu, count_pending,  data, blen, timeout_list,
                   cmd_id, sequence_number);

        timespec cur_time;
        get_monotonic_time(cur_time);
        fpu.last_updated = cur_time;
    }


}

}

}
