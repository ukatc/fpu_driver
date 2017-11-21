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
// NAME time_utils.cpp
//
// provides some common utilities for timing
//
////////////////////////////////////////////////////////////////////////////////

#include "canlayer/time_utils.h"


namespace mpifps
{

namespace canlayer
{


timespec get_monotonic_time(timespec& now)
{
    clock_gettime(CLOCK_MONOTONIC, &now);
    return now;
}


timespec time_add(const timespec& time_a,
                  const timespec& time_b)
{
    timespec sum;
    // FIXME: Overflow is not handled
    set_normalized_timespec(sum,
                            time_a.tv_sec + time_b.tv_sec,
                            time_a.tv_nsec + time_b.tv_nsec);

    return sum;
}

timespec time_sub(const timespec& time_a,
                  const timespec& time_b)
{
    timespec diff;
    // FIXME: Underflow is not handled
    set_normalized_timespec(diff,
                            time_a.tv_sec - time_b.tv_sec,
                            time_a.tv_nsec - time_b.tv_nsec);

    return diff;
}



// returns true if tm_a represents a smaller time than tm_b
bool time_smaller(const timespec&  tm_a, const timespec& tm_b)
{
    return (timespec_compare(tm_a, tm_b) < 0);
}

// returns true if tv_a represents the same time as tv_b
bool time_equal(const timespec& tm_a, const timespec& tm_b)
{
    return (timespec_compare(tm_a, tm_b) == 0);
}


// returns true if tv_a represents a smaller or equal
// time than tv_b
bool time_smaller_equal(const timespec& tm_a, const timespec& tm_b)
{
    return (timespec_compare(tm_a, tm_b) <= 0);
}


// computes the time to wait from cur_time to
// next_timeout, clipping the result to zero
// if the timeout has already passed.
timespec time_to_wait(const timespec& cur_time,
                      const timespec& next_timeout)
{
    const struct timespec zero_wait = {/* .tv_sec = */ 0,
                                       /* .tv_nsec = */ 0};

    // check whether next_timeout is already in the past.
    if (time_smaller_equal(next_timeout, cur_time))
    {
        return zero_wait;
    }
    else
    {
        return time_sub(next_timeout, cur_time);
    }

}


}

}
