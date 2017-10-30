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
#include <time.h>


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

    
}

// this function needs to be called in locked (mutex-protected) state,
// and returns whether any state changed occured that warrants a
// return from waitForState()
bool FPUArray::inTargetState(E_WaitTarget tstate)
{
    // if there is any unreported error
    // (such as a collision or a connection failure)
    // return true regardless of specific query.
    if (grid_state.new_error)
    {
        grid_state.new_error = false;
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
    case MOVEMENT_ABORTED  
        if (grid_state.new_aborted)
        {
            grid_state.new_aborted = false;
            return true;
        }
    break;

    case ANY_CHANGE
        // this checks if any aspect of the state has changed since
        // the last call.  This would include e.g. any position report
        // from any FPU.  Apart from debugging, this can however be
        // useful for tasks such as plotting positions in real time.
        if (grid_state.state_changed)
        {
            grid_state.state_changed = false;
            return true;
        }
    break;

    default:
        // all targets should be covered
        ASSERT(false);

            
    }
    
}

// returns true if tv_a represents a smaller time than tv_b
bool time_smaller(time_spec& tm_a, time_spec& tm_b)
{
    return ((tm_a.tv_sec < tm_b.tv_sec)
            || ( (tm_a.tv_sec == tm_b.tv_sec)
                 && (tm_a.tv_nsec < tm_b.tv_nsec)));
                 
}

// returns true if tv_a represents a smaller time than tv_b
bool time_equal(time_spec& tm_a, time_spec& tm_b)
{
    return ( (tm_a.tv_sec == tm_b.tv_sec)
             && (tm_a.tv_nsec == tm_b.tv_nsec));
                 
}


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
                                        

        fpu_state.cmd_timeout = tout_val;
        fpu_state.pending_command = pending_command;

        // the following adjustments keep the invariant
        // that cached_timeout keeps the minimum value,
        // and cached_timeout_multiplicity the
        // number of times it occurs.

        if (time_smaller(tout_val, cached_timeout))
        {
            // cache is invalidated by setting a smaller value
            // we update the cached value
            cached_timeout = tout_val;
            cached_timeout_multiplicity = 1;
        }
        else if (time_equal(tout_val, cached_timeout))
        {
            // the new value equals the existing
            // minimum
            if (! (was_active && was_equal_minimum))
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
