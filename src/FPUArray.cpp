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
// NAME FPUArray.cpp
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////


#include <pthread.h>
#include <math.h>
#include <errno.h>
#include <time.h>

#include <cassert>


#include "GridState.h"
#include "canlayer/time_utils.h"
#include "canlayer/sync_utils.h"
#include "canlayer/FPUArray.h" 
#include "canlayer/handleFPUResponse.h"
#include "canlayer/handleTimeout.h"

#ifdef DEBUG
#include <stdio.h>
#endif

namespace mpifps
{

namespace canlayer
{




FPUArray::FPUArray(int nfpus)
{

    // TODO: check if any condition variables
    // really need dynamic initialization.

    FPUGridState.count_timeout = 0;
    FPUGridState.count_pending     = 0;

    memset(FPUGridState.Counts, 0,
           sizeof(FPUGridState.Counts));

    // for the beginning, we don't know the FPU states
    FPUGridState.Counts[FPST_UNKNOWN] = nfpus;

    for (int i=0; i < MAX_NUM_POSITIONERS; i++)
    {

        t_fpu_state& fpu_state = FPUGridState.FPU_state[i];

        fpu_state.was_zeroed                = false;
        fpu_state.is_locked                 = false;
        fpu_state.state                     = FPST_UNKNOWN;
        fpu_state.previous_state            = FPST_UNKNOWN;
        fpu_state.pending_command_set       = 0;
        for(int i=0; i < MAX_NUM_TIMEOUTS; i++)
        {
            fpu_state.cmd_timeouts[i].tout_val       = TimeOutList::MAX_TIMESPEC;
            fpu_state.cmd_timeouts[i].cmd_code       = CCMD_NO_COMMAND;
        }
        fpu_state.num_active_timeouts = 0;
        fpu_state.last_updated.tv_sec       = 0;
        fpu_state.last_updated.tv_nsec      = 0;
        fpu_state.timeout_count             = 0;
        fpu_state.last_command              = CCMD_NO_COMMAND;
        fpu_state.last_status               = ER_OK;
        fpu_state.sequence_number           = 0;
        // the values below are not valid, they need proper
        // initialization from a physical fpu response.
        fpu_state.alpha_steps               = 0;
        fpu_state.beta_steps                = 0;
        fpu_state.alpha_datum_switch_active = false;
        fpu_state.beta_datum_switch_active  = false;
        fpu_state.at_alpha_limit            = false;
        fpu_state.beta_collision            = false;
        fpu_state.direction_alpha           = DIRST_UNKNOWN;
        fpu_state.direction_beta            = DIRST_UNKNOWN;
        fpu_state.num_waveforms             = 0;
        fpu_state.waveform_valid            = false;
        fpu_state.waveform_ready            = false;
        fpu_state.waveform_reversed         = false;

    }


    num_fpus = nfpus;
    num_trace_clients = 0;
    FPUGridState.num_queued = 0;
}



FPUArray::~FPUArray()
{
    // destroy condition variable
    pthread_cond_destroy(&cond_state_change);

    // destroy grid state mutex
    pthread_mutex_destroy(&grid_state_mutex);
}


E_DriverErrCode FPUArray::initialize()
{

    // Initialize cond_state_change condition variable with monotonic
    // clock option. (Otherwise, system clock adjustments could cause
    // bugs).
    if (condition_init_monotonic(cond_state_change) != 0)
    {
        return DE_ASSERTION_FAILED;
    }

    return DE_OK;
}

E_DriverErrCode FPUArray::deInitialize()
{

#if FPUARRAY_USE_MONOTONIC_CLOCK    
    
#pragma message "fix up hang on destruction of condition variable"
    /* when the Python process ends, the destructr is called and this
       condition variable is destroyed. However this causes the Python
       control thread to hang.

       The issue appeared after moving the deinitalization here (it
       was automatic before). However this is the proper place
       because we cannot use RAII and exceptions, so we need
       initializer / deinitializer methods.
    */
    pthread_cond_destroy(&cond_state_change);
#endif
    
    return DE_OK;
}



// this function returns a thread-safe copy of the current state of
// the FPU grid.  The important aspect is that the returned value is
// strictly isolated from ongoing concurrent changes in the reading
// thread.
//
// This first implementation simply copies the internal state, which
// likely returns much more information than needed.
//
// FIXME: if relevant, replace internal type with slimmed down
// information which is actually relevant for callers.
E_GridState FPUArray::getGridState(t_grid_state& out_state) const
{

    pthread_mutex_lock(&grid_state_mutex);
    // we simply copy the internal state
    out_state = FPUGridState;
    pthread_mutex_unlock(&grid_state_mutex);

    return getGridStateSummary(out_state);
}


// function that checks whether all FPUs have an refreshed status
// timestamp.
bool check_all_fpus_updated(int num_fpus,
                            t_grid_state& old_grid_state,
                            const t_grid_state& grid_state) 
{
    bool all_updated = true;
    for(int i = 0; i < num_fpus; i++)
    {
        if (grid_state.FPU_state[i].state != FPST_LOCKED)
        {
            timespec new_timestamp = grid_state.FPU_state[i].last_updated;
            timespec old_timestamp = old_grid_state.FPU_state[i].last_updated;
            if (time_equal(old_timestamp, new_timestamp))
            {
                all_updated = false;
                break;
            }
                
        }
    }
    return all_updated;
}


void FPUArray::incSending()
{
    pthread_mutex_lock(&grid_state_mutex);
    FPUGridState.num_queued++;
    assert(! ((FPUGridState.num_queued == 0)
              && (FPUGridState.count_pending == 0) ));
    pthread_mutex_unlock(&grid_state_mutex);
    
}

int  FPUArray::countSending() const
{
    int rv;
    pthread_mutex_lock(&grid_state_mutex);
    rv = FPUGridState.num_queued;
    pthread_mutex_unlock(&grid_state_mutex);
    
    return rv;
}


void FPUArray::decSending()
{
    pthread_mutex_lock(&grid_state_mutex);
    FPUGridState.num_queued--;
    if ( (FPUGridState.num_queued == 0)
         && (FPUGridState.count_pending == 0) )
    {
        pthread_cond_broadcast(&cond_state_change);
    }
    pthread_mutex_unlock(&grid_state_mutex);
    
}



E_GridState FPUArray::waitForState(E_WaitTarget target, t_grid_state& reference_state,
                                   double &max_wait_time, bool &cancelled) const
{

    E_GridState sum_state = GS_UNKNOWN;
    
    
    struct timespec abs_wait_time = {/* .tv_sec = */ 0,
                                     /* .tv_nsec = */ 0};
    
    if (max_wait_time >= 0)
    {
        struct timespec wait_start_time;
        get_monotonic_time(wait_start_time);
        
        struct timespec ts_wait_time;
        ts_wait_time.tv_sec = int(floor(max_wait_time));
        const double scale_nanosec = 1e9;
        ts_wait_time.tv_nsec = int(floor(scale_nanosec * (max_wait_time - floor(max_wait_time))));    
        // get normalized absolute time
        abs_wait_time = time_add(wait_start_time, ts_wait_time);
    }
    cancelled = false;
    
    
    // if we want to get signaled on any minor changes,
    // we increment a special counter to trigger
    // additional event notifications.
    if (target == TGT_ANY_CHANGE)
    {
        // Note, this specific target is very expensive because
        // of the resulting frequent copying of large
        // data structures - it is probably better to define a
        // more specific target.
        num_trace_clients++;
    }
    pthread_mutex_lock(&grid_state_mutex);

    const unsigned long count_timeouts = reference_state.count_timeout;

    bool got_value = false;
    while (! got_value)
    {

        // note this test functions *must* me be called
        // in 'locked' (mutex-protected) state!
        sum_state = getStateSummary_unprotected();
        
        // if a time-out occurs and qualifies, we return early.
        // (the counter can wrap around - no problem!)
        const bool new_timeout_triggered = ( (target & TGT_TIMEOUT) &&
            (count_timeouts != FPUGridState.count_timeout));


        // If all FPUs have been updated, that might be
        // enough.
        const bool all_updated = ((target & GS_ALL_UPDATED) &&
                             check_all_fpus_updated(num_fpus,
                                                    reference_state,
                                                    FPUGridState));

        const bool target_reached = inTargetState(sum_state, target);

        const bool driver_unconnected = FPUGridState.driver_state != DS_CONNECTED;

        const bool end_wait = (target_reached
                               || new_timeout_triggered
                               || all_updated
                               || driver_unconnected);


        if (end_wait)
        {
            // We copy the internal state.  This is a comperatively
            // expensive operation which as of today (2017) has a
            // latency of about 25 microseconds. The reason we return
            // a copy here is that it can't change under the hood
            // later, therefore using the copy does not need any
            // locking.
            reference_state = FPUGridState;
            got_value = true;
            break;
        }
        else
        {
            if (max_wait_time < 0)
            {
                int rv = pthread_cond_wait(&cond_state_change,
                                           &grid_state_mutex);
                assert(rv == 0);
            } else
            {
                int rv = pthread_cond_timedwait(&cond_state_change,
                                           &grid_state_mutex, &abs_wait_time);
                if (rv == ETIMEDOUT)
                {
                    cancelled = true;
                    // grab current state
                    reference_state = FPUGridState;
                    got_value = true;
                    break; // exit while loop
                }
                else
                {
                    // all other values fail assertion
                    assert(rv == 0);
                }
            }
        }
       
    }
    pthread_mutex_unlock(&grid_state_mutex);
    if (target == TGT_ANY_CHANGE)
    {
        // This switches the frequent generation of
        // condition variable signals ('tracing') off if no more
        // clients are listening.
        // In general, tracing should only be used
        // for debugging because moving all the
        // return data becomes costly when done for
        // every single event generated by every single FPU.
        num_trace_clients--;
    }

    if (max_wait_time > 0)
    {
        // compute remaining wait time, and return it in in/out parameter
        struct timespec return_time;
        get_monotonic_time(return_time);

        const struct timespec rest_wait_time = time_to_wait(return_time,
                                                       abs_wait_time);
        max_wait_time = (double(rest_wait_time.tv_sec)
                         + 1.0e-9 * rest_wait_time.tv_nsec);
        
        
    }

    return sum_state;
}

// this function needs to be called in locked (mutex-protected) state,
// and returns whether any state changed occured that warrants a
// return from waitForState()
bool FPUArray::inTargetState(E_GridState sum_state,
                             E_WaitTarget tstate) const
{
    // if there is any unreported error
    // (such as a collision or a connection failure)
    // return true regardless of specific query.

    // check if the driver is working and connected -
    // if not, the state cannot change.
    E_DriverState dstate = FPUGridState.driver_state;
    if (dstate != DS_CONNECTED)
    {
        return true;
    }


    /* The next wait conditionals are some meta
       targets which do not depend only on the grid state,
       but also on the driver state. That's a bit hacky
       and should be streamlined somehow.
    */
    if (tstate == TGT_ANY_CHANGE)
    {
        return true;
    }

    
    // return whether we wait for no more
    // pending commands - this is needed
    // if the caller merely wants to get new
    // info from the grid, instead of a state change.
    if ((tstate & TGT_NO_MORE_PENDING)
        && (FPUGridState.count_pending == 0)
        && (FPUGridState.num_queued == 0))
    {
        return true;
    }


    if ((tstate & TGT_NO_MORE_MOVING)
        && (FPUGridState.count_pending == 0)
        && (FPUGridState.num_queued == 0)
        && (FPUGridState.Counts[FPST_DATUM_SEARCH] == 0)
        && (FPUGridState.Counts[FPST_READY_FORWARD] == 0)
        && (FPUGridState.Counts[FPST_READY_BACKWARD] == 0)
        && (FPUGridState.Counts[FPST_MOVING] == 0))
        
    {
        return true;
    }

    
    // Now, we check whether the bit mask for
    // the state we are looking at matches
    // the return value.

    return ((sum_state & tstate) != 0);



}


bool FPUArray::isLocked(int fpu_id) const
{
    bool is_locked = false;
    pthread_mutex_lock(&grid_state_mutex);
    is_locked = (FPUGridState.FPU_state[fpu_id].state == FPST_LOCKED);
    pthread_mutex_unlock(&grid_state_mutex);

    return is_locked;
}



// sets pending command for one FPU, increments the "pending"
// grid-global counter.

void FPUArray::setPendingCommand(int fpu_id, E_CAN_COMMAND pending_cmd, timespec tout_val,
                                TimeOutList& timeout_list)
{
    pthread_mutex_lock(&grid_state_mutex);


    t_fpu_state& fpu = FPUGridState.FPU_state[fpu_id];

    // Currently, we do assume that commands expect a response
    // independently from the FPU state, even if the FPU is
    // locked. The only exception are configureMotion commands where
    // both the first and last flags are not set.


    add_pending(fpu, fpu_id, pending_cmd, tout_val, timeout_list,
                FPUGridState.count_pending);
    // if tracing is active, signal state change
    if (num_trace_clients > 0)
    {
        pthread_cond_broadcast(&cond_state_change);
    }


    pthread_mutex_unlock(&grid_state_mutex);
}

// sets last command for a FPU

void FPUArray::setLastCommand(int fpu_id, E_CAN_COMMAND last_cmd)
{
    pthread_mutex_lock(&grid_state_mutex);


    t_fpu_state& fpu = FPUGridState.FPU_state[fpu_id];
    fpu.last_command = last_cmd;
    fpu.last_status = ER_OK_UNCONFIRMED;

    // if tracing is active, signal state change
    // to waitForState() callers.
    if (num_trace_clients > 0)
    {
        printf("T"); fflush(stdout);
        printf("¡3");fflush(stdout);
        pthread_cond_broadcast(&cond_state_change);
    }
    pthread_mutex_unlock(&grid_state_mutex);
}



// updates state for all FPUs which did
// not respond in time

// this function adjusts each FPU
// record which has a pending command with a
// timeout value that is equal or smaller than
// the cur_time value.
//
// The count of timeouts is correspongly
// increased. After finishing the search,
// the cond_state_change condition variable
// is signalled if any timeout was found.
//


void FPUArray::processTimeouts(timespec cur_time, TimeOutList& tout_list)
{
    timespec next_key;
    pthread_mutex_lock(&grid_state_mutex);

    int old_count_pending = FPUGridState.count_pending;
    bool state_count_changed = false;

    while (true)
    {
        TimeOutList::t_toentry toentry;
        // get absolute timeout value
        
        next_key = tout_list.getNextTimeOut();

        assert(next_key.tv_sec > 0);
        
        if (time_smaller(cur_time, next_key))
        {
            break;
        }
        toentry = tout_list.pop();

        // check whether entry was found
        if (toentry.id == -1)
        {
            printf("no more entries found, break\n");
            break;
        }

        assert(toentry.id < num_fpus);
            
        int fpu_id = toentry.id;
        

        t_fpu_state& fpu = FPUGridState.FPU_state[fpu_id];

        const E_FPU_STATE old_state = fpu.state;
        
        // remlove entries which are expired, and adjust pending count
        timespec next_timeout = expire_pending(fpu, fpu_id,
                                               cur_time,
                                               FPUGridState.count_pending,
                                               FPUGridState.count_timeout);
        
        const E_FPU_STATE new_state = fpu.state;
        if (old_state != new_state)
        {
            FPUGridState.Counts[old_state]--;
            FPUGridState.Counts[new_state]++;
            state_count_changed = true;
        }
        
        // re-insert smallest time-out of remaining pending commands
        tout_list.insertTimeOut(fpu_id, next_timeout);
        

    }
    


    // signal any waiting control threads if
    // the grid state has changed
    if (((FPUGridState.count_pending == 0)
         && (FPUGridState.num_queued == 0))  ||
        ((old_count_pending > FPUGridState.count_pending)
         && ((num_trace_clients > 0)))
       || state_count_changed )
    {
        pthread_cond_broadcast(&cond_state_change);
    }
    pthread_mutex_unlock(&grid_state_mutex);

}


// This function sets the global state of
// the CAN driver. It allows to notify
// callers of waitForState() when any relevant
// change of the system happens, such as a lost
// socket connection.
void FPUArray::setDriverState(E_DriverState const dstate)
{
    pthread_mutex_lock(&grid_state_mutex);
    E_DriverState old_state = FPUGridState.driver_state;        
    FPUGridState.driver_state = dstate;
    
    if (old_state != dstate)
    {
        pthread_cond_broadcast(&cond_state_change);
    }
    pthread_mutex_unlock(&grid_state_mutex);

}


E_DriverState FPUArray::getDriverState() const
{
    E_DriverState retval;

    pthread_mutex_lock(&grid_state_mutex);
    retval = FPUGridState.driver_state;
    pthread_mutex_unlock(&grid_state_mutex);
    return retval;
}


E_GridState FPUArray::getStateSummary_unprotected() const
{
    // get the summary state of the grid member variable.
    // (This relies on that all FPU updates
    // do mirror the global counters correctly).

    return getGridStateSummary(FPUGridState);

}

E_GridState FPUArray::getStateSummary() const
{
    E_GridState sum_state;
    pthread_mutex_lock(&grid_state_mutex);

    sum_state = getStateSummary_unprotected();

    pthread_mutex_unlock(&grid_state_mutex);

    return sum_state;

}

void FPUArray::dispatchResponse(const t_address_map& fpu_id_by_adr,
                                const int gateway_id,
                                const uint8_t bus_id,
                                const uint16_t can_identifier,
                                const t_response_buf& data,
                                const int blen,
                                TimeOutList& tout_list)
{

    int fpu_busid = can_identifier & 0x7f;
    
#ifdef DEBUG2
#define PRINT_BYTES    
#ifdef PRINT_BYTES    
    int priority = can_identifier >> 7;
    printf("dispatching response: gateway_id=%i, bus_id=%i,can_identifier=%i,"
           "priority=%i, fpu_busid=%i, data[%i] =",
           gateway_id, bus_id, can_identifier, priority, fpu_busid, blen);
#endif    

    assert(gateway_id < MAX_NUM_GATEWAYS);

    int nbytes = blen;
    if (nbytes > MAX_CAN_PAYLOAD_BYTES)
    {
        printf("WARNING: blen too long");
        nbytes = MAX_CAN_PAYLOAD_BYTES;
    }
    if (nbytes < 0)
    {
        printf("WARNING: blen negative");
        nbytes = 0;
    }
#ifdef PRINT_BYTES    
    for (int i=0; i < nbytes; i++)
    {
        printf(" %02i", data[i]);
    }
    printf("\n");
#endif    
#endif

#if (CAN_PROTOCOL_VERSION == 1)
    if (blen != 8)
    {
#ifdef DEBUG
        printf("wrong message length (blen=%i) for version 1 - ignored.\n", blen);
#endif        
        return;
    }
#endif    

    // fpu_busid is a one-based index
    if (fpu_busid == 0)
    {
#ifdef DEBUG
        printf("fpu_busid zero, ignored\n");
#endif
        return;
    }
    if (fpu_busid > FPUS_PER_BUS)
    {
#ifdef DEBUG
        printf("fpu_busid too large (%i), ignored\n", fpu_busid);
#endif
        return;
    }
    if (bus_id >= BUSES_PER_GATEWAY)
    {
#ifdef DEBUG
        printf("bus_id too large (%i), ignored\n", bus_id);
#endif
        return;
    }

    // The redundant fpu_id is part of the version 1 protocol,
    // version 2 gets rid of this.
    if (data[0] != fpu_busid)
    {
#ifdef DEBUG
            printf("RX invalid message for protocol 1: payload fpu_busid (%i) does"
                   " not match fpu_busid from canid (%i) - ignored.\n",
                   data[0], fpu_busid);
#endif
            return;
    }

    // fpu_busid uses a one-based index here, this is
    // deliberate and reflected in the array size.
    int fpu_id = fpu_id_by_adr[gateway_id][bus_id][fpu_busid];
    if ((fpu_id > num_fpus) || (fpu_id > MAX_NUM_POSITIONERS))
    {
#ifdef DEBUG2
        printf("fpu_id too large (%i), ignored\n", fpu_id);
#endif
        return;
    }

    
    pthread_mutex_lock(&grid_state_mutex);
    
    {

        // get canid of FPU (additional ids might be used
        // to report state of the gateway)


  
        // Unwrap response, and adjust FPU state according to
        // that.

        const t_fpu_state oldstate = FPUGridState.FPU_state[fpu_id];

        canlayer::handleFPUResponse(fpu_id, FPUGridState.FPU_state[fpu_id], data, blen,
                                    tout_list, FPUGridState.count_pending);


        // update global state counters
        t_fpu_state newstate = FPUGridState.FPU_state[fpu_id];

        // remember previous state
        if(newstate.state != oldstate.state)
        {
            FPUGridState.FPU_state[fpu_id].previous_state = oldstate.state;
        }
        
        FPUGridState.Counts[oldstate.state]--;
        FPUGridState.Counts[newstate.state]++;

        // The state of the grid can change when *all* FPUs have left
        // an old state, or *at least one* has entered a new state.
        bool state_transition = ((oldstate.state != newstate.state)
                                 && ((FPUGridState.Counts[oldstate.state] == 0)
                                     || (FPUGridState.Counts[newstate.state] == 1)));

        // if no more commands are pending or tracing is active,
        // signal a state change to waitForState() callers.
        if ( ((FPUGridState.num_queued == 0) && (FPUGridState.count_pending == 0))
             || state_transition
             || (num_trace_clients > 0) )
        {
            pthread_cond_broadcast(&cond_state_change);
        }

    } // end of locked block
    pthread_mutex_unlock(&grid_state_mutex);

#ifdef PRINT_BYTES    
    printf(" (dispatch_response): OK\n");
#endif    
    
    



}


timespec get_min_pending(const t_fpu_state& fpu)
{
   // get earliest previous time-out value
    assert(fpu.num_active_timeouts >= 0);
#ifdef DEBUG3
    printf("get_min_pending(): fpu.num_active_timeouts = %i\n",
           fpu.num_active_timeouts);
#endif
    timespec min_val = TimeOutList::MAX_TIMESPEC;
    for (int i=0; i < fpu.num_active_timeouts; i++)
    {
        timespec cmp_val = fpu.cmd_timeouts[i].tout_val;
        if ( time_smaller(cmp_val, min_val))
        {
            min_val = cmp_val;
        }
    }
    return min_val;
}

void add_pending(t_fpu_state& fpu, int fpu_id, E_CAN_COMMAND cmd_code,
                 const timespec& new_timeout,
                 TimeOutList& timeout_list, int &count_pending)
{
#ifdef DEBUG3
    printf("fpu #%i: adding cmd code %i\n", fpu_id, cmd_code);
#endif
    // assert this command is not yet pending
    assert( ((fpu.pending_command_set >> cmd_code) & 1) == 0);
    assert(fpu.num_active_timeouts < (MAX_NUM_TIMEOUTS-1));
    assert(fpu.num_active_timeouts >= 0);

    // add command to pending set
    fpu.pending_command_set |= ((unsigned int)1) << cmd_code;

    // get earliest previous time-out value
    timespec min_val = get_min_pending(fpu);

    tout_entry new_entry;
    new_entry.cmd_code = cmd_code;
    new_entry.tout_val = new_timeout;
        
    fpu.cmd_timeouts[fpu.num_active_timeouts++] = new_entry;
    // if the new value is smaller than the previous ones,
    // overwrite the time-out list entry for this fpu
    if (time_smaller(new_timeout, min_val))
    {
        timeout_list.insertTimeOut(fpu_id, new_timeout);
    }

    count_pending++;
    assert(fpu.num_active_timeouts >= 1);

}

void remove_pending(t_fpu_state& fpu, int fpu_id,
                    E_CAN_COMMAND cmd_code, E_MOC_ERRCODE cmd_status,
                    TimeOutList& timeout_list, int &count_pending)
{
    // ignore if a command was already removed by time-out expiration

    if (fpu.num_active_timeouts == 0)
    {
#ifdef DEBUG2
    printf("fpu #%i:  cmd code %i was already removed by time-out\n", fpu_id, cmd_code);
#endif
        return;
    }
    assert(cmd_code != CCMD_NO_COMMAND);

    if ( ((fpu.pending_command_set >> cmd_code) & 1) == 0)
    {
#ifdef DEBUG2
    printf("fpu #%i:  cmd code %i was already removed\n", fpu_id, cmd_code);
#endif
        return;
    }

#ifdef DEBUG2
    printf("fpu #%i: removing cmd code %i %i ==> %i\n", fpu_id, cmd_code,
           fpu.pending_command_set,
           fpu.pending_command_set & ~(((unsigned int)1) << cmd_code));
#endif

    // iterate list to find entry and value which is to be removed
    timespec removed_val;
    int del_index;
    bool found = false;
    for(int i = 0; i < fpu.num_active_timeouts; i++)
    {
        if (fpu.cmd_timeouts[i].cmd_code == cmd_code)
        {
            del_index = i;
            removed_val = fpu.cmd_timeouts[i].tout_val;
            found = true;
            break;
        }
    }
    assert(found);
    // move all/any following entries to previous position
    for (int i = del_index; i < (fpu.num_active_timeouts - 1); i++)
    {
        fpu.cmd_timeouts[i] = fpu.cmd_timeouts[i+1];
    }

    // overwrite last entry (not necessary, but could help debugging)
    tout_entry del_entry;
    del_entry.cmd_code = CCMD_NO_COMMAND;
    del_entry.tout_val = TimeOutList::MAX_TIMESPEC;
    fpu.cmd_timeouts[fpu.num_active_timeouts - 1] = del_entry;
    
    fpu.num_active_timeouts--;
    
    
    // remove command from pending set
    fpu.pending_command_set &= ~(((unsigned int)1) << cmd_code);
    fpu.last_command = cmd_code;
    fpu.last_status = cmd_status;

    // get earliest remaining time-out value.
    // This can be MAX_TIMESPEC as well, no problem.
    timespec new_min_val = get_min_pending(fpu);

    if (time_smaller(removed_val, new_min_val))
    {
        timeout_list.insertTimeOut(fpu_id, new_min_val);
    }

    count_pending--;
    assert(fpu.num_active_timeouts >= 0);
#ifdef DEBUG2
    printf("fpu #%i:  remove_pending: new pending count is %i\n", fpu_id, count_pending);
#endif

}

timespec expire_pending(t_fpu_state& fpu, int fpu_id, const timespec& expiration_time,
                        int &count_pending, unsigned long &count_timeouts)
{

    // remove all commands in the pending set which have a timeout
    // value before or equal to the expiration time, adjust pending
    // count, and return the remaining minimum time.
    printf("entering expire_pending, fpuid=%i, fpu.num_active_timeouts = %i\n",
           fpu_id, fpu.num_active_timeouts);
    assert(fpu.num_active_timeouts >= 0);

    if (fpu.num_active_timeouts == 0)
    {
        return TimeOutList::MAX_TIMESPEC;
    }
    
    const int old_active_timeouts = fpu.num_active_timeouts;
    int read_index = 0;
    int write_index = 0;
    while ( read_index < old_active_timeouts)
    {
        bool preserve = time_smaller(expiration_time,
                                     fpu.cmd_timeouts[read_index].tout_val);
        if (preserve)
        {
            if (read_index > write_index)
            {
                fpu.cmd_timeouts[write_index] = fpu.cmd_timeouts[read_index];
            }
            write_index ++;
        }
        else
        {
            uint8_t cmd_code = fpu.cmd_timeouts[read_index].cmd_code;
            fpu.pending_command_set &= ~(((unsigned int)1) << cmd_code);
            fpu.last_command = static_cast<E_CAN_COMMAND>(cmd_code);
            fpu.last_status = ER_TIMEDOUT;
            
            count_pending--;
            // Note: the counter below wraps and this is intended,
            // it is an unsigned value which will wrap around
            // and is only compared against change.
            count_timeouts++;
            printf("decrementing num_active_timeouts %i -> %i\n",
                   fpu.num_active_timeouts, fpu.num_active_timeouts -1);
            fflush(stdout);
            fpu.num_active_timeouts--;
            // fix state if necessary
            canlayer::handleTimeout(fpu_id, fpu, static_cast<E_CAN_COMMAND>(cmd_code));
        }
        read_index++;
        
    }

    if (old_active_timeouts > write_index)
    {
        tout_entry del_entry;
        del_entry.cmd_code = CCMD_NO_COMMAND;
        del_entry.tout_val = TimeOutList::MAX_TIMESPEC;

        for (int i = write_index; i < old_active_timeouts; i++)
        {
            // overwrite last entry (not necessary, but could help debugging)
            fpu.cmd_timeouts[i] = del_entry;
    
        }
    }

    
    
    assert(fpu.num_active_timeouts >= 0);

    const timespec new_min_val = get_min_pending(fpu);

    if (fpu.num_active_timeouts == 0)
    {
        assert( (new_min_val.tv_sec == TimeOutList::MAX_TIMESPEC.tv_sec)
              && (new_min_val.tv_nsec == TimeOutList::MAX_TIMESPEC.tv_nsec));
    }
    printf("expirePending() next timeout: %li/%li\n",
           new_min_val.tv_sec, new_min_val.tv_nsec);

    printf("exiting expire_pending, n_active = %i\n", fpu.num_active_timeouts);
    return new_min_val;
}


}

}
