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

#ifndef HANDLE_TIMEOUT_H
#define HANDLE_TIMEOUT_H

#include "FPUState.h"
#include "canlayer/I_CAN_Command.h"
#include "GridDriverConfig.h"

namespace mpifps
{
    namespace canlayer
    {

        void handleTimeout(const GridDriverConfig config, int fpu_id, t_fpu_state& fpu, E_CAN_COMMAND cmd_id);

    }
}

#endif
