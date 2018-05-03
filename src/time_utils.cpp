////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 UKRI. See file "LICENSE" for license information.
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client samplepn


//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME time_utils.cpp
//
// provides some common utilities for timing
//
////////////////////////////////////////////////////////////////////////////////

#include "canlayer/time_utils.h"
#include <cassert>
#include <limits.h>

namespace mpifps
{

namespace canlayer
{


timespec get_monotonic_time(timespec& now)
{
    clock_gettime(CLOCK_MONOTONIC, &now);
    return now;
}

double get_realtime()
{
    timespec now;
    double dnow;
    clock_gettime(CLOCK_REALTIME, &now);
    dnow = now.tv_sec + now.tv_nsec * 1e-9;
    return dnow;
}


timespec time_add(const timespec& time_a,
                  const timespec& time_b)
{
    timespec sum;
    const long nano = 1000000000;
    // treat value TIME_T_MAX like a floating point inf symbol
    if ((time_a.tv_sec == TIME_T_MAX) || (time_b.tv_sec == TIME_T_MAX))
    {
        sum.tv_sec = TIME_T_MAX;
        sum.tv_nsec = nano -1;
        return sum;
    }

    // check for overflow
    {
        long a = time_a.tv_sec;
        long b = time_b.tv_sec;

        assert( ( (b >= 0) && (a < (TIME_T_MAX - b)))
                || ((b < 0) && ( a > (LONG_MIN - b))));
    }

    set_normalized_timespec(sum,
                            time_a.tv_sec + time_b.tv_sec,
                            time_a.tv_nsec + time_b.tv_nsec);

    return sum;
}

timespec time_sub(const timespec& time_a,
                  const timespec& time_b)
{
    timespec diff;

    const long nano = 1000000000;

    // treat value TIME_T_MAX like a floating point inf symbol

    // inf - inf = undefined
    assert(! ((time_a.tv_sec == TIME_T_MAX) && (time_b.tv_sec == TIME_T_MAX)));

    // inf - x = inf
    if ((time_a.tv_sec == TIME_T_MAX) && (time_b.tv_sec != TIME_T_MAX))
    {
        diff.tv_sec = TIME_T_MAX;
        diff.tv_nsec = nano -1;
        return diff;
    }

    // x - inf = 0
    if ((time_a.tv_sec != TIME_T_MAX) && (time_b.tv_sec == TIME_T_MAX))
    {
        diff.tv_sec = 0;
        diff.tv_nsec = 0;
        return diff;
    }

    // check for overflow
    {
        long a = time_a.tv_sec;
        long b = time_b.tv_sec;

        assert( ( (b >= 0) && (a > (LONG_MIN + b)))
                || ((b < 0) && ( a < (TIME_T_MAX + b))));
    }



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
              /* .tv_nsec = */ 0
    };

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
