////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 UKRI. See file "LICENSE" for license information.
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client samplepn


//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME sync_utils.C
//
// provides some common utilities for timing
//
////////////////////////////////////////////////////////////////////////////////

#include "ethercan/sync_utils.h"

namespace mpifps
{

namespace ethercanif
{


int condition_init_monotonic(pthread_cond_t& cond)
{
    // initialize the condition variable to
    // use the Linux monotonic clock
    pthread_condattr_t cond_attr;
    int err = 0;
    err = pthread_condattr_init(&cond_attr);
    // non-zero return values are specified as
    // logical errors, so the caller should just
    // wrap this into ann assert.
    if (err != 0)
    {
        goto exit;
    }
    // set the Linux monotonic clock
    err = pthread_condattr_setclock(&cond_attr, CLOCK_MONOTONIC);
    if (err != 0)
    {
        goto exit;
    }
    err = pthread_cond_init(&cond, &cond_attr);
    if (err != 0)
    {
        goto exit;
    }
    err = pthread_condattr_destroy(&cond_attr);
    if (err != 0)
    {
        goto exit;
    }

exit:
    return err;
}

}

}
