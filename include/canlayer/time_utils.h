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

    namespace canlayer
{

// the following three  definitions are from Linux-4.3.6
// kernel sources - they are not included in time.h.

const time_t TIME_T_MAX	= (time_t)((1UL << ((sizeof(time_t) << 3) - 1)) - 1);


static inline int timespec_equal(const struct timespec *a,
                                 const struct timespec *b)
{
    return (a->tv_sec == b->tv_sec) && (a->tv_nsec == b->tv_nsec);
}


/*
 * lhs < rhs:  return <0
 * lhs == rhs: return 0
 * lhs > rhs:  return >0
 */
static inline int timespec_compare(const struct timespec& lhs,
                                   const struct timespec& rhs)
{
    if (lhs.tv_sec < rhs.tv_sec)
        return -1;
    if (lhs.tv_sec > rhs.tv_sec)
        return 1;
    return lhs.tv_nsec - rhs.tv_nsec;
}

static inline void set_normalized_timespec(struct timespec &new_val,
        const time_t tv_sec, const long tv_nsec)
{
    new_val.tv_sec = tv_sec + tv_nsec / 1000000000;
    new_val.tv_nsec = tv_nsec % 1000000000;
    // NOTE: correctness for negative nsec values
    // depends on the C99 standard definition for
    // the modulo operator (negative dividend ->
    // negative remainder). THIS IS DIFFERENT
    // FROM PYTHON.
}

// get current monotonic system time.
// Monotonic means that even at leap seconds,
// it keeps increasing, so it is suitable
// for measuring time-outs.
timespec get_monotonic_time(timespec& now);

// Adds two timespecs. The sum must be representable,
// otherwise undefined behavior happens.
timespec time_add(const timespec& time_a,
                  const timespec& time_b);


// Subtracts two timespecs. The result must be representable,
// otherwise undefined behavior happens.
timespec time_sub(const timespec& time_a,
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

}

#endif
