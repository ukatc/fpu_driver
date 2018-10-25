////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client sample
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME handleFPUResponse.h
//
// This function implements the handler for FPU CAN responses.
// It is not a class method, stressing that the FPU status data
// has any "hidden" private fields.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef HANDLE_PING_FPU_RESPONSE_H
#define HANDLE_PING_FPU_RESPONSE_H

#include "FPUState.h"
#include "EtherCANInterfaceConfig.h"
#include "ethercan/CAN_Command.h"
#include "ethercan/TimeOutList.h"

namespace mpifps
{
namespace ethercanif
{
void handle_PingFPU_response(const EtherCANInterfaceConfig&config,
                             const int fpu_id,
                             t_fpu_state& fpu,
                             int &count_pending,
                             const t_response_buf&data,
                             const int blen, TimeOutList&  timeout_list,
                             const E_CAN_COMMAND cmd_id,
                             const uint8_t response_status,
                             const E_MOC_ERRCODE response_errcode,
                             const timespec& cur_time);
}
}

#endif
