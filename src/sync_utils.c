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


namespace mpifps
{

    int conditon_init_monotonic(pthread_cond_t& cond)
    {
        // initialize the condition variable to
        // use the Linux monotonic clock
        pthread_condattr_t cond_attr;
        int err = 0;
        err = pthread_condattr_init(&cond_attr);
        // non-zero return values are specified as
        // logical errors, so the caller should just
        // wrap this into ann ASSERT.
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
        err = pthread_cond_init(&cond_queue_append, &cond_attr);
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
