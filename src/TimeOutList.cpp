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
// NAME TimeOutList.cpp
//
// This class implements an ordered list if Time_out values,
// which can be safely accessed from multiple threads
//
////////////////////////////////////////////////////////////////////////////////

#include <time.h>
//#include <string.h>		/// strerror
#include <pthread.h>

#include "DriverConstants.h"
#include "canlayer/TimeOutList.h"


namespace mpifps
{

namespace canlayer
{


// A general note on time-out handling: When the driver
// performs a poll on the receiving end, it needs
// to wait until any FPU times out, that is the
// FPU with the command which has the smallest time-out value.

// The most frequent operations in terms of time-outs
// are insertion of a new value, finding a minimum value,
// and deletion of a
// value where we received a timely response.
// Therefore, we tentatively maximize these cases
// using an O(1) algorithm on the cost of worst case search time
// which is O(N) .
//
// More efficient data structures are possible but might not
// be trivial to implement.

const timespec TimeOutList::MAX_TIMESPEC = {/* .tv_sec = */ TIME_T_MAX,
                                          /* .tv_nsec = */ 999999999
};


TimeOutList::TimeOutList()
{
    for(int i = 0; i < MAX_NUM_POSITIONERS; i++)
    {
        TimeOutsByID[i] = MAX_TIMESPEC;
    }
    // initialize the cache
    cached_minimum_multiplicity = MAX_NUM_POSITIONERS;
    cached_minimum = MAX_TIMESPEC;
    minimum_index_lbound = 0;

}


// insert new value into list
void TimeOutList::insertTimeOut(int const id, timespec new_val)
{

    // we make use of the circumstance that
    // messages are mostly sent in bursts, therefore
    // timeout values are normally very similar, und
    // use a quantization of 5 milliseconds.
    // This has the advantage that a memory traversal
    // searching for a new minimum is triggered less
    // frequently - in most cases we just need
    // to update the multplicity count.

    // as an edge case, the new value can be MAX_TIMEOUT,
    // which means in practice we clear the entry.

    if (! time_equal(new_val, MAX_TIMESPEC))
    {
        const long quant_nsec = 5000000; // 5 milliseconds
        long nano_secs =  (((new_val.tv_nsec + quant_nsec)
                            / quant_nsec)
                           * quant_nsec);
        // normalize value
        set_normalized_timespec(new_val, new_val.tv_sec, nano_secs);
    }


    pthread_mutex_lock(&list_mutex);

    const timespec old_val = TimeOutsByID[id];
    TimeOutsByID[id] = new_val;
    
#ifdef DEBUGT
    printf("insertTimeOut(): new timeout for FPU #%i = %li / %li\n",
           id, TimeOutsByID[id].tv_sec, TimeOutsByID[id].tv_nsec);
#endif    

    // update invariants

    // true if the old value equalled the cached minimum
    bool const was_equal_minimum = time_equal(old_val,
                                   cached_minimum);

    // true if new value is now equal to minimum
    bool const is_equal_minimum = time_equal(new_val,
                                  cached_minimum);

    // true if new value is now smaller than minimum
    bool const is_smaller_minimum = time_smaller(new_val,
                                    cached_minimum);


    // the following adjustments keep the invariant
    // that cached_minimum keeps the minimum value,
    // and cached_minimum_multiplicity the
    // number of times it occurs.

    if (is_smaller_minimum)
    {
        // cache is invalidated by setting a smaller value,
        // therefore we update the cached value.
        cached_minimum = new_val;
        // we know this is the only value,
        // because the minimum was not set to it before
        cached_minimum_multiplicity = 1;

        minimum_index_lbound = id;
    }
    else if (is_equal_minimum)
    {
        // the new value equals the existing
        // minimum but didn't before
        if (! was_equal_minimum)
        {
            // we increment the count, if it was not
            // already included
            cached_minimum_multiplicity += 1;
            
            if (id < minimum_index_lbound)
            {
                minimum_index_lbound = id;
            }
        }
    }
    else if (was_equal_minimum)
    {
        // this is the most likely case.
        // we overwrote a minimal timeout value with
        // a larger value, so we need to decrement
        // the cache multiplicity (triggering
        // a full minimum search in the minKey() method,
        // once the count goes to zero).
        cached_minimum_multiplicity -= 1;
        if (cached_minimum_multiplicity == 0)
        {
            // we lose information about the minimum
            // place
            minimum_index_lbound = 0;
        }
    }


    pthread_mutex_unlock(&list_mutex);


}

// clear value from list
void TimeOutList::clearTimeOut(int fpu_id)
{
    insertTimeOut(fpu_id, MAX_TIMESPEC);
}

// get current minimum key (timeout value)
// Because this function is called often
// (before each call to poll() on the receiving thread),
// and a full search traverses more memory than
// fits in Intel Xeon's L1 cache, we cache the minimum
// value and its first index position.
// Note: this method is NOT thread-safe and is therefore
// internal.
const timespec TimeOutList::minKey()
{
    timespec min_val = MAX_TIMESPEC;

#ifdef DEBUG    
    clock_t t0 = clock();
#endif


    // first we try to use the cache

#if 0
    if (cached_minimum_multiplicity > 0)
#else        
    if (false)
#endif        
    {
        // cached value is still valid
        if (time_smaller(cached_minimum, min_val))
        {
            min_val = cached_minimum;
        }
        // otherwise, max_time is used, which
        // was assigned by initialization
    }
    else
    {
        // we need to search for the minimum value and
        // refresh the count how often it occurs.
        for(int i = 0; i < MAX_NUM_POSITIONERS; i++)
        {
            // using a linear search seems less efficient
            // than some combination of lists and a
            // priority queue, however this does not happens
            // very often, and makes good use of the CPU cache.

            timespec const next_timeout = TimeOutsByID[i];
            if (time_smaller(next_timeout, min_val))
            {
                min_val = next_timeout;
                // finding a new minimum refreshes
                // the multiplicity and cache
                cached_minimum = next_timeout;
                cached_minimum_multiplicity = 1;
                minimum_index_lbound = i;
            }
            else if (time_equal(min_val, next_timeout))
            {
                // a recurring value, we increment the
                // multiplicity count
                cached_minimum_multiplicity += 1;
            }

        }
    }

#ifdef DEBUG    
    clock_t t1 = clock();
    clock_t td = t1 - t0;
    if (td > 100)
    {
        printf("minKey : took %li usec\n", td);
    }
#endif
    return min_val;
}


// get and remove item with minimum value
TimeOutList::t_toentry TimeOutList::pop()
{
    t_toentry result;

    pthread_mutex_lock(&list_mutex);

    // the lock needs to include the minimum search
    // to avoid any possible race condition -
    // otherwise we could search for a minimum
    // that has been removed in the meantime.

    // get the current minimum
    timespec minval = minKey();

    if (time_equal(minval, MAX_TIMESPEC))
    {
        // guess the list was empty....
        result.id = -1;
        result.val = minval;
    }
    else
    {

        // we need to search for the index of a minimum element
        // but we can use minimum_index_lbound as lower bound
        int i = 0;
        bool found = false;
        for (i = minimum_index_lbound; i < MAX_NUM_POSITIONERS; i++)
        {
            if (time_equal(TimeOutsByID[i],
                           minval))
            {
                found = true;
                break;
            }
        }
        result.val = minval;
        if (found)
        {
            result.id = i;
            TimeOutsByID[i] = MAX_TIMESPEC;
            minimum_index_lbound = i + 1;
        }
        else
        {
            result.id = -1;
        }

    }
    pthread_mutex_unlock(&list_mutex);

    return result;
}


// this function retrieves the minimum time-out
// time for each FPU in the FPU grid which
// has any pending command. If no time-out
// is found, it returns MAX_TIMESPEC.
const timespec TimeOutList::getNextTimeOut()
{

    timespec min_val;

    pthread_mutex_lock(&list_mutex);
    min_val = minKey();
    pthread_mutex_unlock(&list_mutex);

    return min_val;
}


}

}
