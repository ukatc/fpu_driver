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

#include <stdint.h>
#include <memory>
#include <vector>

//#include <stdio.h>
//#include <string.h>		/// strerror
#include <pthread.h>
//#include "stdlib.h"		/// system("/bin/stty raw");
//#include "stdbool.h"	/// bool
//#include <unistd.h>

#include "DriverState.h"
#include "I_CAN_Command.h"
#include "E_CAN_COMMAND.h"


namespace mpifps
{

using std::unique_ptr;

class CommandPool {
      public:

        CommandPool(int nfpus){
            num_fpus = nfpus;
        };
        
        ~CommandPool(){};
        
        // initializes the pool, allocating
        // all the required memory for once
        E_DriverErrCode initialize();
        
        // method which provides a new CAN command
        // instance for the given command type.
        // If the pool is temporarily empty, the
        // method blocks until an instance is available.
    template <typename T>
    unique_ptr<T> provideInstance(E_CAN_COMMAND cmd_type);

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
#endif
