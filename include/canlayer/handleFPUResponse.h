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

#ifndef HANDLE_FPU_RESPONSE_H
#define HANDLE_FPU_RESPONSE_H

#include "FPUState.h"
#include "GridDriverConfig.h"
#include "canlayer/I_CAN_Command.h"
#include "canlayer/TimeOutList.h"

namespace mpifps
{
namespace canlayer
{
    void handleFPUResponse(const GridDriverConfig config,
                           int fpu_id, t_fpu_state& fpu,
                           const t_response_buf& data,
                           const int blen, TimeOutList& timeout_list,
                           int &count_pending);

}
}

#endif
