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

void handleTimeout(const GridDriverConfig &config, int fpu_id, t_fpu_state& fpu, E_CAN_COMMAND cmd_id)
{
    timespec cur_time;

    get_monotonic_time(cur_time);

    // this counter is a unsigned 16-bit value which can wrap
    // around. It must therefore *always* be compared for inequality!
    fpu.timeout_count++;

    switch (cmd_id)
    {
    case CCMD_PING_FPU   :
        fpu.ping_ok = false;
        LOG_RX(LOG_ERROR, "%18.6f : RX WARNING: pingFPU command  timed out for FPU #%i!.\n",
               canlayer::get_realtime(),
               fpu_id);
        break;

    case CCMD_CONFIG_MOTION   :
        break;

    case CCMD_EXECUTE_MOTION  :
        LOG_RX(LOG_ERROR, "%18.6f : RX ERROR: executeMotion command  timed out for FPU #%i!.\n",
               canlayer::get_realtime(),
               fpu_id);
        fpu.state = FPST_RESTING;
        fpu.ping_ok = false;
        break;

    case CCMD_ABORT_MOTION    :
        fprintf(stderr, "CRITICAL ERROR: ABORT_MOTION timed out for FPU #%i!\n", fpu_id);

        LOG_RX(LOG_ERROR, "%18.6f : RX CRITICAL ERROR: ABORT_MOTION timed out for FPU #%i!.\n",
               canlayer::get_realtime(),
               fpu_id);
        fpu.ping_ok = false;
        break;

    case CCMD_RESET_FPU       :
        fpu.state = FPST_UNKNOWN;
        fpu.ping_ok = false;
        break;

    case CCMD_FIND_DATUM :
        fpu.state = FPST_UNINITIALIZED;
        fpu.ping_ok = false;
        fprintf(stderr, "Error: findDatum timed out for FPU #%i\n", fpu_id);

        LOG_RX(LOG_ERROR, "%18.6f : RX ERROR: findDatum() timed out for FPU #%i!.\n",
               canlayer::get_realtime(),
               fpu_id);
        break;


    case CCMD_ENABLE_BETA_COLLISION_PROTECTION:
        fpu.state = fpu.previous_state;
        LOG_RX(LOG_ERROR, "%18.6f : RX ERROR: enableBetaCollisionProtection() command timed out for FPU #%i!.\n",
               canlayer::get_realtime(),
               fpu_id);
        break;

    case CCMD_FREE_BETA_COLLISION:
        LOG_RX(LOG_ERROR, "%18.6f : RX ERROR: freeBetaCollision() command timed out for FPU #%i!.\n",
               canlayer::get_realtime(),
               fpu_id);
        fpu.state = fpu.previous_state;
        fpu.ping_ok = false;
        break;

    default:
        // invalid command, ignore
        LOG_RX(LOG_ERROR, "%18.6f : RX WARNING: FPU #%i: time-out for command code %i ignored.\n",
               canlayer::get_realtime(),
               fpu_id,
               cmd_id);
        break;

    }

}

}

}
