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

#include "time_utils.h"


namespace mpifps
{

timespec get_monotonic_time(timespec& now)
{
    clock_gettime(CLOCK_MONOTONIC, &now);
}


timespec add_time(const timespec& time_a,
                   const timespec& time_b)
{
    timespec sum = time_a;
    // FIXME: replace with Linux function!
    sum.tv_sec += time_b.tv_sec;
    sum.tv_nsec += time_b.tv_nsec;
    return sum;
}


// returns true if tv_a represents a smaller time than tv_b
bool time_smaller(const timespec&  tm_a, const timespec& tm_b)
{
    // FIXME: replace with Linux function!
    return ((tm_a.tv_sec < tm_b.tv_sec)
            || ( (tm_a.tv_sec == tm_b.tv_sec)
                 && (tm_a.tv_nsec < tm_b.tv_nsec)));
                 
}

// returns true if tv_a represents the same time as tv_b
bool time_equal(const timespec& tm_a, const timespec& tm_b)
{
    // FIXME: normalize both arguments!!!
    return ( (tm_a.tv_sec == tm_b.tv_sec)
             && (tm_a.tv_nsec == tm_b.tv_nsec));
                 
}


// returns true if tv_a represents a smaller or equal
// time than tv_b
bool time_smaller_equal(const timespec& tm_a, const timespec& tm_b)
{
    // FIXME: replace with Linux function!
    return ( (tm_a.tv_sec < tm_b.tv_sec)
             || ((tm_a.tv_sec == tm_b.tv_sec)
                 && (tm_a.tv_nsec <= tm_b.tv_nsec)));
                 
}


// computes the time to wait from cur_time to
// next_timeout, clipping the result to zero
// if the timeout has already passed.
timespec time_to_wait(const timespec& cur_time,
                      const timespec& next_timeout)
{
    timespec ttw = next_timeout;

    // FIXME: use Linux functions!
    ttw.tv_sec -= cur_time.tv_sec;
    if (ttw.tv_sec < 0)
        ttw.tv_sec = 0;

    ttw.tv_nsec -= cur_time.tv_nsec;
    if (ttw.tv_nsec < 0)
        ttw.tv_nsec = 0;

    
}



}
