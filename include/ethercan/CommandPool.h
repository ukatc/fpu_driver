// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 UKRI. See file "LICENSE" for license information.
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client sample
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME CommandPool.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#ifndef COMMAND_POOL_H
#define COMMAND_POOL_H

#include <cassert>
#include <memory>
#include <vector>

#include <pthread.h>

#include "../InterfaceState.h"
#include "../ErrorCodes.h"
#include "../EtherCANInterfaceConfig.h"

#include "E_CAN_COMMAND.h"
#include "CAN_Command.h"

namespace mpifps
{

namespace ethercanif
{

using std::unique_ptr;

class CommandPool
{
public:

    explicit CommandPool(const EtherCANInterfaceConfig &config_vals):
        config(config_vals)
    {
    };

    ~CommandPool() {};

    // initializes the pool, allocating
    // all the required memory for once
    E_EtherCANErrCode initialize();


    // de-initialize pool and return memory.
    // Most only be called when driver is shut down.
    E_EtherCANErrCode deInitialize();

    // method which provides a new CAN command
    // instance for the given command type.
    // If the pool is temporarily empty, the
    // method blocks until an instance is available.
    //    template <typename T>
    //    unique_ptr<T> provideInstance(E_CAN_COMMAND cmd_type);
    //
    template<typename T>
    inline unique_ptr<T> provideInstance()
    {
        unique_ptr<CAN_Command> ptr;
        // get the command code for that class
        E_CAN_COMMAND cmd_code = T::getCommandCode();
        assert(cmd_code > 0);
        assert(cmd_code < NUM_CAN_COMMANDS);

        pthread_mutex_lock(&pool_mutex);
        while (pool[cmd_code].empty())
        {
            // wait until a command instance is in the pool
            // Waiting should almost never happen because
            // there is a surplus of instances - if
            // we ever get a dead-lock here, we have a memory
            // leak of command instances.
#ifdef DEBUG
            printf("CommandPool::provideInstance() : waiting for resource\n");
#endif
            pthread_cond_wait(&cond_pool_add, &pool_mutex);
        }

        ptr = std::move(pool[cmd_code].back());
        pool[cmd_code].pop_back();
        pthread_mutex_unlock(&pool_mutex);

        unique_ptr<T> ptrT;
        // if this assert fails, it is a logical error.
        assert(ptr.get() != nullptr);
        ptrT.reset(dynamic_cast<T*>(ptr.release()));
        return ptrT;
    }

    // method which recycles an instance that
    // is no longer needed into the memory pool so that it can
    // be used later without requiring a new
    // allocation.
    void recycleInstance(unique_ptr<CAN_Command>& cmdptr);


private:
    typedef std::vector<unique_ptr<CAN_Command>> t_cmdvec;

    const EtherCANInterfaceConfig config;
    t_cmdvec pool[NUM_CAN_COMMANDS+1]; // numbers are one-based
    pthread_mutex_t pool_mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t cond_pool_add = PTHREAD_COND_INITIALIZER;
};

}

}
#endif
