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
// NAME time_utils.h
//
// provides some common utilities for timing
//
////////////////////////////////////////////////////////////////////////////////

#ifndef TIME_UTILS_H
#define TIME_UTILS_H
#include <time.h> 

namespace mpifps
{

    // get current monotonic system time.
    // Monotonic means that even at leap seconds,
    // it keeps increasing, so it is suitable
    // for measuring time-outs.
    timespec get_monotonic_time(timespec& now);

    timespec add_time(const timespec& time_a,
                       const timespec& time_b);

    bool time_smaller(const timespec& tm_a, const timespec& tm_b);

    bool time_equal(const timespec& tm_a, const timespec& tm_b);

    bool time_smaller_equal(const timespec& tm_a, const timespec& tm_b);
    // computes the time to wait from cur_time to
    // next_timeout, clipping the result to zero
    // if the timeout has already passed.
    timespec time_to_wait(const timespec& cur_time,
                           const timespec& next_timeout);

}

#endif
