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

#include <time.h> 

namespace mpifps
{

    // get current monotonic system time.
    // Monotonic means that even at leap seconds,
    // it keeps increasing, so it is suitable
    // for measuring time-outs.
    void timespec get_monotonic_time(timespec& now);

    timespec add_time(const timespec& time_a,
                       const timespec& time_b);

    bool time_smaller(timespec& const tm_a, timespec& const tm_b);

    bool time_equal(timespec& const tm_a, timespec& const tm_b);

    bool time_smaller_equal(timespec& const tm_a, timespec& const tm_b)
    // computes the time to wait from cur_time to
    // next_timeout, clipping the result to zero
    // if the timeout has already passed.
    timespec time_to_wait(timespec& const cur_time,
                           timespec& const next_timeout);

}
