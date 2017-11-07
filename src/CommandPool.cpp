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

#include "CommandPool.h"

namespace mpifps
{



E_DriverErrCode CommandPool::initialize()
{
    E_DriverErrCode rval = DRIVER_NOT_INITIALISED;

    ASSERT(num_fpus > 0);
    for (int i = 0; i < NUM_CAN_COMMANDS; i++)
    {
        int capacity=0;
        const int cap_broadcast = 10;
        const int cap_individual = num_fpus * 10;
        const int cap_wform = num_fpus * MAX_SUB_COMMANDS;
        
        switch (i)
        {
            // broadcast commands
        case BROADCAST       :
            capacity = cap_broadcast;
            break;
            
            // waveform table
        case CONFIG_MOTION   :
            capacity = cap_wform;
            break;
            
            // individual commands
        case PING_FPU        :
        case MOVE_DATUM_OFF  :
        case MOVE_DATUM_ON   :
        case EXECUTE_MOTION  :
        case REPEAT_MOTION   :
        case REVERSE_MOTION  :
        case REQUEST_STATUS  :
        case REPORT_POSITIONS:
        case ASSIGN_POSITION :
        case ABORT_MOTION    :
        case UNTANGLE_FPU    :
        case CLEAR_COLLISION :
        case CHECK_INTEGRITY :
        case RESET_FPU       :
        case LOCK_UNIT       :
        case UNLOCK_UNIT     :
            capacity = cap_individual;
        default:
            // logical error
            ASSERT(false);
        }

        pthread_mutex_lock(&pool_mutex);
        // FIXME: This can thow bad_alloc if the
        // system is very low on memory.
        pool[i].reserve(capacity);
        unique_ptr<I_CAN_Command> ptr;
        for (int c = 0; c < capacity; c++)
        {
            switch (i)
            {
            case PING_FPU        :
                ptr = new PingCommand();
                pool[i].push_back(ptr);
                break;

            case CONFIG_MOTION        :
                ptr = new ConfigureMotionCommand();
                pool[i].push_back(ptr);
                break;
            case MOVE_DATUM_OFF        :
                ptr = new MoveDatumOffCommand();
                pool[i].push_back(ptr);
                break;
            case MOVE_DATUM_ON        :
                ptr = new MoveDatumOnCommand();
                pool[i].push_back(ptr);
                break;
                
            default:
                // FIXME: add any missing constructors
                //ASSERT(0);
                
            }
        }
        pthread_mutex_unlock(&pool_mutex);

        return OK;

        
    }
}

//template<typename T>
unique_ptr<T> CommandPool::provideInstance(E_CAN_COMMAND cmd_type)
{
    unique_ptr<I_CAN_Command> ptr = null;

    pthread_mutex_lock(&pool_mutex);
    while (pool[cmd_type].empty())
    {
        // wait until a command instance is in the pool
        // Waiting should almost never happen because
        // there is a surplus of instances - if
        // we ever get a dead-lock here, we have a memory
        // leak of command instances.
        pthread_cond_wait(&pool_cond_add, &pool_mutex);
    }
    
    ptr = pool[cmd_type].pop_back();
    pthread_mutex_unlock(&pool_mutex);

    return ptr;
    // return dynamic_cast<T>(ptr);
}

// Adds a used command to the pool again,
// and if any thread is waiting, notify it
// that there are command buffers available
// again. 
void CommandPool::recycleInstance(unique_ptr<I_CAN_Command>& cmdptr)
{
    pthread_mutex_lock(&pool_mutex);
    bool was_empty = pool[cmd_type].empty();
    pool[cmd_type].push_back(cmdptr);
    cmd_ptr = null;
    // if we just added to an empty pool, notify one waiting thread
    // that it can make progress.
    if (was_empty)
    {
        pthread_cond_signal(&pool_cond_add);
    }
    pthread_mutex_unlock(&pool_mutex);
}

}
