////////////////////////////////////////////////////////////////////////////////
// ESO - VLT Project
//
// Copyright 2017 E.S.O,
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// Pablo Gutierrez 2017-07-22  created CAN client sample
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client samplepn


//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME sync_utils.h
//
// provides some common utilities for timing
//
////////////////////////////////////////////////////////////////////////////////

#include <time.h> 
#include <pthread.h>

namespace mpifps
{

    // initializes a pthreads condition variable
    // so that it uses the Linux monotonic clock.
    // returns a non-zero value if the initialization
    // failed because passing an invalid argument.
    //
    // (The return value should likely be checked
    // by an ASSERT.)
    int conditon_init_monotonic(pthread_cond_t& cond);


}
