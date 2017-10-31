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
// NAME FPU_CAN_driver.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////


#include <stdio.h>
#include <string.h>		/// strerror
#include <pthread.h>
#include <unistd.h>
#include <std>
#include "time_utils.h"


#include "FPUArray.h" // defines thread-safe structure of FPU state info
    
namespace mpifps
{

// this function returns a thread-safe copy of the current state of
// the FPU grid.  The important aspect is that the returned value is
// strictly isolated from ongoing concurrent changes in the reading
// thread.
//
// This first implementation simply copies the internal state, which
// likely returns much more information than needed.
// FIXME: replace internal type with slimmed down information
// which is actually relevant for callers.
void FPUarray::getGridState(t_grid_state& out_state)
{

    pthread_mutex_lock(&grid_state_mutex);
    // we simply copy the internal state
    out_state = FPUGridState;
    pthread_mutex_unlock(&grid_state_mutex);
}


void FPUarray::waitForState(E_WaitTarget target, t_grid_state& out_state)
{

    bool got_value = false;
    // if we want to get signaled on any minor changes,
    // we increment a special counter to trigger
    // additional event notifications.
    if (target == ANY_CHANGE)
    {
        num_trace_clients++;
    }
    pthread_mutex_lock(&grid_state_mutex);

    while (! got_value)
    {

        // note this test function *must* me be called
        // in 'locked' state!
        if (! inTargetState(target))
        {
            pthread_cond_wait(&cond_state_change,
                              &grid_state_mutex);
        }
        else
        {
            // we copy the internal state
            out_state = FPUGridState;
            got_value = true;
        }
    }
    pthread_mutex_unlock(&grid_state_mutex);
    if (target == ANY_CHANGE)
    {
        num_trace_clients--;
    }

    
}

// this function needs to be called in locked (mutex-protected) state,
// and returns whether any state changed occured that warrants a
// return from waitForState()
bool FPUArray::inTargetState(E_WaitTarget tstate)
{
    // if there is any unreported error
    // (such as a collision or a connection failure)
    // return true regardless of specific query.
    // FIXME: That will need some refinement for
    // error recovery (noving out of collisions etc).
    E_DriverState state = grid_state.driver_state;
    if ((state == ABORTED)
        || (state == NO_CONNECTION)
        || (state == UNINITIALISED))
    {
        return true;
    }

    // report collisions early
    if ( (tstate == FINISHED)
         && (count_collision > 0))
    {
        return true;
    }
    
    switch(tstate){

    case INITIALISED:
        return (grid_state.count_initialised +
                grid_state.count_locked +
                grid_state.count_timeout) == num_fpus;
        break;
        
    case AT_DATUM:
        return (grid_state.count_datum +
                grid_state.count_locked +
                grid_state.count_timeout) == num_fpus;
        break;
        
    case READY_TO_MOVE:
        return (grid_state.count_ready +
                grid_state.count_locked +
                grid_state.count_timeout) == num_fpus;
        break;
        
        
    case MOVEMENT_FINISHED 
        return (grid_state.count_finished +
                grid_state.count_locked +
                grid_state.count_timeout) == num_fpus;
        break;

    case ANY_CHANGE
        // this returns on any signal on the condition variable, that
        // is if any aspect of the state has changed since the last
        // call.  This would include e.g. any position report from any
        // FPU.  Apart from debugging, this can however be useful for
        // tasks such as plotting positions in real time.
        return true;
    break;

    default:
        // all targets should be covered
        ASSERT(false);

            
    }
    
}

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

// An alternative would be a priority queue implemented
// as a binary heap, which has O(1) time for finding
// the minimum value, and O(log(N)) time for
// both insertions and deletions.

// TODO: The above should be reconsidered when doing performance
// testing. Any changes should be purely internal.
    
    




// this function retrieves the minimum time-out
// time for each FPU in the FPU grid which
// has any pending command. If no time-out
// is found, it returns the passed default value.
// Because this function is called often
// (before each call to poll() on the receiving thread ),
// and a full search traverses lots of memory,
// we cache the minimum value.
time_spec FPUArray::getNextTimeOut(time_spec max_time)
{

    time_spec min_val = max_time;
    
    pthread_mutex_lock(&grid_state_mutex);

    // first we try to use the cache

    if (cached_timeout_multiplicity > 0)
    {
        // cached value is still valid
        if (time_smaller(cached_timeout, max_time))
        {
            min_val = cached_timeout;
        }
        // otherwise, max_time is used, which
        // was assigned by initialization
    }
    else
    {
        // we need to search for the minimum value and
        // by the way also count how often it occurs.
        const t_fpu_grid& fpu_state = FPUGridState.FPU_state;
        for(int i = 0; i < num_fpus; i++)
        {
            if (fpu_state[i].pending_command != NoCommand)
            {
                next_timeout = fpu_state[i].cmd_timeout;
                if (time_smaller(next_timeout, min_val))
                {
                    min_val = next_timeout;
                    // finding a new minimum refreshes
                    // the multiplicity and cache
                    cached_timeout = next_timeout;
                    cached_timeout_multiplicity = 1;
                }
                else if (time_equal(min_val, next_timeout))
                {
                    // a recurring value, we increment the count
                    cached_timeout_multiplicity += 1;
                }
            }
        }
    }
    pthread_mutex_unlock(&grid_state_mutex);

    return min_val;

    
};

FPUArray::setNextTimeOut(int fpu_id,
                         E_CAN_COMMAND pending_command,
                         time_spec tout_val)
    {

        // we make use of the circumstance that
        // timeout values are normally very similar, und
        // use a quantization of 250 microseconds.

        const long quant_nsec = 250000;
        long nano_secs = tout_val.nsec;
        tout_val.nsec = (((nano_secs + quant_nsec)
                          / quant_nsec)
                         * quant_nsec);
        
        
        pthread_mutex_lock(&grid_state_mutex);

        t_fpu_state& fpu_state = FPUGridState.FPU_state[fpu_id];

        // the old value equals the cached minimum
        bool was_equal_minimum = time_equal(cached_timeout,
                                            fpu_state.cmd_timeout);
        // the FPU had an active command
        bool was_active fpu_state.pending_command != NoCommand;

        bool is_active == pending_command != NoCommand;
                                        

        fpu_state.cmd_timeout = tout_val;
        fpu_state.pending_command = pending_command;

        // the following adjustments keep the invariant
        // that cached_timeout keeps the minimum value,
        // and cached_timeout_multiplicity the
        // number of times it occurs.

        if (time_smaller(tout_val, cached_timeout))
        {
            // cache is invalidated by setting a smaller value,
            // therefore we update the cached value.
            cached_timeout = tout_val;
            cached_timeout_multiplicity = 1;
        }
        else if (time_equal(tout_val, cached_timeout))
        {
            // the new value equals the existing
            // minimum and increases the number of
            // FPUs waiting until that time
            if ((! (was_active && was_equal_minimum))
                && is_active)
            {
                // we increment the count, if it was not
                // already included
                cached_timeout_multiplicity += 1;
            }
        }
        else if ((was_active) && (was_equal_minimum))
        {
            // this is the most probable case.
            // we overwrote an active timeout with
            // a larger value, so we need to decrement
            // the cache multiplicity (triggering
            // a full minimum search once the count goes to zero).
            cached_timeout_multiplicity -= 1;        
        }
    
        
    
        pthread_mutex_unlock(&grid_state_mutex);
    };


FPUArray::clearTimeOut(int fpu_id)
{
    setNextTimeOut(fpu_id, NoCommand, MAX_TIMEOUT);
}


}
