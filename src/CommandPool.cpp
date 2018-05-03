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
// NAME CommandPool.cpp
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#include <cassert>

#include "DriverConstants.h"
#include "canlayer/CommandPool.h"
// alphabetically sorted below
#include "canlayer/commands/AbortMotionCommand.h"
#include "canlayer/commands/ConfigureMotionCommand.h"
#include "canlayer/commands/EnableBetaCollisionProtectionCommand.h"
#include "canlayer/commands/ExecuteMotionCommand.h"
#include "canlayer/commands/FindDatumCommand.h"
#include "canlayer/commands/FreeBetaCollisionCommand.h"
#include "canlayer/commands/GetErrorAlphaCommand.h"
#include "canlayer/commands/GetErrorBetaCommand.h"
#include "canlayer/commands/GetStepsAlphaCommand.h"
#include "canlayer/commands/GetStepsBetaCommand.h"
#include "canlayer/commands/PingFPUCommand.h"
#include "canlayer/commands/RepeatMotionCommand.h"
#include "canlayer/commands/ResetFPUCommand.h"
#include "canlayer/commands/ReverseMotionCommand.h"
#include "canlayer/commands/SetUStepLevelCommand.h"

#ifdef DEBUG
#include <stdio.h>
#endif

namespace mpifps
{

namespace canlayer
{


E_DriverErrCode CommandPool::initialize()
{

    assert(config.num_fpus > 0);
    pthread_mutex_lock(&pool_mutex);
    bool allocation_error = false;
    try
    {
        // This starts to count with 1 because 0 is no
        // actual command.
        for (int i = 1; i < NUM_CAN_COMMANDS; i++)
        {
            int capacity=0;
            const int cap_broadcast = 10;
            const int cap_individual = config.num_fpus * 10;
            const int cap_wform = config.num_fpus * MAX_SUB_COMMANDS;

//            printf("CommandPool::initialize(): cap_individual = %i\n", cap_individual);
            switch (i)
            {
            // broadcast commands
            case CCMD_EXECUTE_MOTION       :
            case CCMD_REPEAT_MOTION       :
            case CCMD_REVERSE_MOTION       :
#if CAN_PROTOCOL_VERSION > 1
            case CCMD_CHECK_INTEGRITY       :
#endif
            case CCMD_ABORT_MOTION       :
                capacity = cap_broadcast;
                break;

            // waveform table
            case CCMD_CONFIG_MOTION   :
                capacity = cap_wform;
                break;

                // individual commands
#if CAN_PROTOCOL_VERSION == 1
            case CCMD_GET_ERROR_ALPHA :
            case CCMD_GET_ERROR_BETA :
            case CCMD_GET_STEPS_ALPHA:
            case CCMD_GET_STEPS_BETA:
#else
            case CCMD_LOCK_UNIT       :
            case CCMD_UNLOCK_UNIT     :
            case CCMD_GET_COUNTER_DEVIATION:
            case CCMD_GET_FIRMWARE_VERSION          :
            case CCMD_SET_TIME_STEP                 :
            case CCMD_SET_STEPS_PER_FRAME             :
            case CCMD_FREE_ALPHA_LIMIT_BREACH       :
            case CCMD_ENABLE_ALPHA_LIMIT_PROTECTION :
            case CCMD_ENABLE_MOVE   :
#endif
            case CCMD_RESET_FPU       :
            case CCMD_PING_FPU       :
            case CCMD_ENABLE_BETA_COLLISION_PROTECTION :
            case CCMD_FREE_BETA_COLLISION    :
            case CCMD_SET_USTEP_LEVEL :
            case CCMD_FIND_DATUM :
            case CCMD_RESET_STEPCOUNTER :
            case CCMD_READ_REGISTER   :
                capacity = cap_individual;
                break;

            default:
                // logical error
#ifdef DEBUG
                printf("fatal error: command code %i not found!\n", i);
#endif
                assert(false);
            }

            // This can throw bad_alloc during initialization
            // if the system is very low on memory.
            pool[i].reserve(capacity);
            unique_ptr<I_CAN_Command> ptr;
            for (int c = 0; c < capacity; c++)
            {
                switch (i)
                {
                case CCMD_PING_FPU        :
                    ptr.reset(new PingFPUCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_CONFIG_MOTION        :
                    ptr.reset(new ConfigureMotionCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_EXECUTE_MOTION        :
                    ptr.reset(new ExecuteMotionCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_REVERSE_MOTION        :
                    ptr.reset(new ReverseMotionCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_REPEAT_MOTION        :
                    ptr.reset(new RepeatMotionCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_ABORT_MOTION        :
                    ptr.reset(new AbortMotionCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_RESET_FPU:
                    ptr.reset(new ResetFPUCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_FIND_DATUM        :
                    ptr.reset(new FindDatumCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_GET_STEPS_ALPHA        :
                    ptr.reset(new GetStepsAlphaCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_GET_STEPS_BETA        :
                    ptr.reset(new GetStepsBetaCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_GET_ERROR_ALPHA        :
                    ptr.reset(new GetErrorAlphaCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_GET_ERROR_BETA        :
                    ptr.reset(new GetErrorBetaCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_ENABLE_BETA_COLLISION_PROTECTION :
                    ptr.reset(new EnableBetaCollisionProtectionCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_FREE_BETA_COLLISION    :
                    ptr.reset(new FreeBetaCollisionCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_SET_USTEP_LEVEL :
                    ptr.reset(new SetUStepLevelCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                default:
#if (CAN_PROTOCOL_VERSION > 1)
#pragma message "FIXME: add any missing constructors"
                    assert(0);
#else
                    assert(1);
#endif

                }
            }
        }
    }
    catch (std::bad_alloc& ba)
    {

        allocation_error = true;
    }
    pthread_mutex_unlock(&pool_mutex);
    if (allocation_error)
    {
        return DE_DRIVER_NOT_INITIALIZED;
    }

    return DE_OK;

}



E_DriverErrCode CommandPool::deInitialize()
{

    assert(config.num_fpus > 0);
    bool allocation_error = false;
    try
    {
        // This starts to count with 1 because 0 is no
        // actual command.
        for (int i = 1; i < NUM_CAN_COMMANDS; i++)
        {

            pool[i].clear();
            // FIXME: std::vector<>::shrink_to_fit can thow bad_alloc if the
            // system is low on memory. This should be caught.
            pool[i].shrink_to_fit();

        }
    }
    catch (std::bad_alloc& ba)
    {

        allocation_error = true;
    }
    if (allocation_error)
    {
        return DE_ASSERTION_FAILED;
    }

    return DE_OK;

}


// Adds a used command to the pool again,
// and if any thread is waiting, notify it
// that there are command buffers available
// again.
void CommandPool::recycleInstance(unique_ptr<I_CAN_Command>& cmd_ptr)
{
    pthread_mutex_lock(&pool_mutex);
    E_CAN_COMMAND cmd_type = cmd_ptr->getInstanceCommandCode();
    bool was_empty = pool[cmd_type].empty();
    pool[cmd_type].push_back(std::move(cmd_ptr));
    cmd_ptr = nullptr;
    // if we just added to an empty pool, notify one waiting thread
    // that it can make progress.
    if (was_empty)
    {
        pthread_cond_signal(&cond_pool_add);
    }
    pthread_mutex_unlock(&pool_mutex);
}



}

}
