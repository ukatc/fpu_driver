// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
// ESO - VLT Project
//
// Copyright 2017 E.S.O,
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// Pablo Gutierrez 2017-07-22  created CAN client sample
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

#include "canlayer/CommandPool.h"
#include "canlayer/DriverConstants.h"
#include "canlayer/commands/ConfigureMotionCommand.h"
#include "canlayer/commands/MoveDatumOnCommand.h"
#include "canlayer/commands/MoveDatumOffCommand.h"
#include "canlayer/commands/PingCommand.h"

namespace mpifps
{

namespace canlayer
{


E_DriverErrCode CommandPool::initialize()
{

    assert(num_fpus > 0);
    pthread_mutex_lock(&pool_mutex);
    bool allocation_error = false;
    try
    {
        for (int i = 0; i < NUM_CAN_COMMANDS; i++)
        {
            int capacity=0;
            const int cap_broadcast = 10;
            const int cap_individual = num_fpus * 10;
            const int cap_wform = num_fpus * MAX_SUB_COMMANDS;

            switch (i)
            {
            // broadcast commands
            case CCMD_RESET_FPU       :
            case CCMD_EXECUTE_MOTION       :
            case CCMD_REPEAT_MOTION       :
            case CCMD_REVERSE_MOTION       :
            case CCMD_REQUEST_STATUS       :
            case CCMD_REPORT_POSITIONS       :
            case CCMD_CLEAR_COLLISION       :
            case CCMD_CHECK_INTEGRITY       :
            case CCMD_PING_FPU       :
            case CCMD_ABORT_MOTION       :
                capacity = cap_broadcast;
                break;

            // waveform table
            case CCMD_CONFIG_MOTION   :
                capacity = cap_wform;
                break;

            // individual commands
            case CCMD_MOVE_DATUM_OFF  :
            case CCMD_MOVE_DATUM_ON   :
            case CCMD_ASSIGN_POSITION :
            case CCMD_UNTANGLE_FPU    :
            case CCMD_LOCK_UNIT       :
            case CCMD_UNLOCK_UNIT     :
                capacity = cap_individual;
            default:
                // logical error
                assert(false);
            }

            // FIXME: This can thow bad_alloc if the
            // system is very low on memory.
            pool[i].reserve(capacity);
            unique_ptr<I_CAN_Command> ptr;
            for (int c = 0; c < capacity; c++)
            {
                switch (i)
                {
                case CCMD_PING_FPU        :
                    ptr.reset(new PingCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_CONFIG_MOTION        :
                    ptr.reset(new ConfigureMotionCommand());
                    pool[i].push_back(std::move(ptr));
                    break;
                case CCMD_MOVE_DATUM_OFF        :
                    ptr.reset(new MoveDatumOffCommand());
                    pool[i].push_back(std::move(ptr));
                    break;
                case CCMD_MOVE_DATUM_ON        :
                    ptr.reset(new MoveDatumOnCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                default:
                    // FIXME: add any missing constructors
                    assert(0);

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
        return DE_DRIVER_NOT_INITIALISED;
    }
    else
    {
        return DE_OK;
    }
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
