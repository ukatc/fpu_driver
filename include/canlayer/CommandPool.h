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
// NAME CommandPool.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#ifndef COMMAND_POOL_H
#define COMMAND_POOL_H

#include <memory>
#include <vector>

#include <pthread.h>

#include "../DriverState.h"


#include "E_CAN_COMMAND.h"
#include "I_CAN_Command.h"

namespace mpifps
{

namespace canlayer
{

using std::unique_ptr;

class CommandPool
{
public:

    CommandPool(int nfpus)
    {
        num_fpus = nfpus;
    };

    ~CommandPool() {};

    // initializes the pool, allocating
    // all the required memory for once
    E_DriverErrCode initialize();

    // method which provides a new CAN command
    // instance for the given command type.
    // If the pool is temporarily empty, the
    // method blocks until an instance is available.
//    template <typename T>
//    unique_ptr<T> provideInstance(E_CAN_COMMAND cmd_type);
//
    template<typename T>
    inline unique_ptr<T> provideInstance(E_CAN_COMMAND cmd_type)
    {
        unique_ptr<I_CAN_Command> ptr;

        pthread_mutex_lock(&pool_mutex);
        while (pool[cmd_type].empty())
        {
            // wait until a command instance is in the pool
            // Waiting should almost never happen because
            // there is a surplus of instances - if
            // we ever get a dead-lock here, we have a memory
            // leak of command instances.
            pthread_cond_wait(&cond_pool_add, &pool_mutex);
        }

        ptr = std::move(pool[cmd_type].back());
        pool[cmd_type].pop_back();
        pthread_mutex_unlock(&pool_mutex);

        unique_ptr<T> ptrT;
        // if this assert fails, it is a logical error.
        assert(ptr.get() != nullptr);
        ptrT.reset(dynamic_cast<T*>(ptr.get()));
        return ptrT;
    }

    // method which recycles an instance that
    // is no longer needed into the memory pool so that it can
    // be used later without requiring a new
    // allocation.
    void recycleInstance(unique_ptr<I_CAN_Command>& cmdptr);

private:
    typedef std::vector<unique_ptr<I_CAN_Command>> t_cmdvec;

    int num_fpus;
    t_cmdvec pool[NUM_CAN_COMMANDS];
    pthread_mutex_t pool_mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t cond_pool_add = PTHREAD_COND_INITIALIZER;
};

}

}
#endif
