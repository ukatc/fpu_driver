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


void handleFPUResponse(const EtherCANInterfaceConfig& config,
                       int fpu_id, t_fpu_state& fpu,
                       const t_response_buf& data,
                       const int blen, TimeOutList& timeout_list,
                       int &count_pending)
{
    E_CAN_COMMAND cmd_id = static_cast<E_CAN_COMMAND>(data[1] & COMMAND_CODE_MASK);
    uint8_t sequence_number = data[0];

    assert(blen <= 8);
    assert(blen >= 4);

    void (*handler) (const EtherCANInterfaceConfig&,
		     const int,
		     t_fpu_state&,
		     int &,
		     const t_response_buf&,
		     const int,
		     TimeOutList&t,
		     const E_CAN_COMMAND,
		     const uint8_t) = nullptr;



    switch (cmd_id)
    {
    case CCMD_CONFIG_MOTION   : handler = &handle_ConfigMotion_response; break;

    case CCMD_EXECUTE_MOTION  : handler = &handle_ExecuteMotion_response; break;

    case CMSG_FINISHED_MOTION: handler = &handle_FinishedMotion_message; break;

    case CCMD_ABORT_MOTION    : handler = &handle_AbortMotion_response; break;

    case CMSG_WARN_CANOVERFLOW    : handler = &handle_WarnCANOverflow_warning; break;

    case CCMD_GET_COUNTER_DEVIATION  : handler = &handle_GetCounterDeviation_response; break;

    case CCMD_PING_FPU        : handler = &handle_PingFPU_response; break;

    case CCMD_REPEAT_MOTION        : handler = &handle_RepeatMotion_response; break;

    case CCMD_REVERSE_MOTION        : handler = &handle_ReverseMotion_response; break;

    case CCMD_RESET_FPU       : handler = &handle_ResetFPU_response; break;

    case CCMD_FIND_DATUM : handler = handle_FindDatum_response; break;

    case CMSG_FINISHED_DATUM : handler = &handle_FinishedDatum_message; break;

    case CMSG_WARN_COLLISION_BETA: handler = &handle_WarnCollisionBeta_warning; break;
	
    case CMSG_WARN_LIMIT_ALPHA: handler = &handle_WarnLimitAlpha_warning; break;

    case CCMD_ENABLE_BETA_COLLISION_PROTECTION: handler = &handle_EnableBetaCollisionProtection_response; break;

    case CCMD_FREE_BETA_COLLISION: handler = &handle_FreeBetaCollision_response; break;

    case CCMD_SET_USTEP_LEVEL: handler = &handle_SetUStepLevel_response; break;

    case CCMD_READ_REGISTER : handler = &handle_ReadRegister_response; break;

    case CCMD_WRITE_SERIAL_NUMBER: handler = &handle_WriteSerialNumber_response; break;

    case CCMD_READ_SERIAL_NUMBER  : handler = & handle_ReadSerialNumber_response; break;

    case CCMD_NO_COMMAND      :
    default:
        // invalid command, ignore
	handler = nullptr;
        LOG_RX(LOG_ERROR, "%18.6f : RX : "
               "invalid message (id # %i) received for FPU %i\n",
               get_realtime(),
               cmd_id,
               fpu_id);
        break;

    }
    if (handler != nullptr)
    {
	(*handler)(config, fpu_id, fpu, count_pending,  data, blen, timeout_list,
		   cmd_id, sequence_number);
    }

    timespec cur_time;
    get_monotonic_time(cur_time);
    fpu.last_updated = cur_time;

}

}

}
