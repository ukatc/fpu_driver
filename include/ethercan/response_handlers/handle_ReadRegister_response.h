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

#ifndef HANDLE_READ_REGISTER_RESPONSE_H
#define HANDLE_READ_REGISTER_RESPONSE_H

#include "FPUState.h"
#include "EtherCANInterfaceConfig.h"
#include "ethercan/CAN_Command.h"
#include "ethercan/TimeOutList.h"
#include "ethercan/decode_CAN_response.h"

namespace mpifps
{
namespace ethercanif
{
void handle_ReadRegister_response(const EtherCANInterfaceConfig&config,
                                  const int fpu_id,
                                  t_fpu_state& fpu,
                                  int &count_pending,
                                  const t_response_buf&data,
                                  const int blen, TimeOutList&  timeout_list,
                                  const E_CAN_COMMAND cmd_id,
                                  const uint8_t sequence_number);
}
}

#endif
