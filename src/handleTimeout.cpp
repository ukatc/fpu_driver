// -*- mode: c++ -*-

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
// NAME handleTimeout.cpp
//
// This function implements specific time-out handling, such as
// cleaning up intermediate state.
//
////////////////////////////////////////////////////////////////////////////////


#include <cassert>

#include "FPUState.h"
#include "canlayer/handleTimeout.h"
#include "canlayer/time_utils.h"
#include "canlayer/FPUArray.h"


#ifdef DEBUG
#include <stdio.h>
#endif

namespace mpifps
{

namespace canlayer
{

void handleTimeout(int fpu_id, t_fpu_state& fpu, E_CAN_COMMAND cmd_id)
{
    timespec cur_time;

    get_monotonic_time(cur_time);

    switch (cmd_id)
    {
    case CCMD_PING_FPU   :
        fpu.ping_ok = false;
        break;

    case CCMD_CONFIG_MOTION   :
        break;

    case CCMD_EXECUTE_MOTION  :
        fpu.state = FPST_RESTING;
        break;

    case CCMD_ABORT_MOTION    :
        printf("CRITICAL ERROR: ABORT_MOTION timed out for FPU #%i!\n", fpu_id);
        break;

    case CCMD_RESET_FPU       :
        fpu.state = FPST_UNKNOWN;
        break;

    case CCMD_FIND_DATUM :
        fpu.state = FPST_UNINITIALIZED;
        printf("Error: findDatum timed out for FPU #%i\n", fpu_id);
        break;


    case CCMD_ENABLE_BETA_COLLISION_PROTECTION:
        fpu.state = fpu.previous_state;
        break;

    case CCMD_FREE_BETA_COLLISION:
        fpu.state = fpu.previous_state;
        break;

    default:
        // invalid command, ignore
        // FIXME: log invalid responses
        break;

    }

}

}

}
