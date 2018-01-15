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
#include <cassert>

#include "canlayer/time_utils.h"

#include "GridState.h"

#include "canlayer/FPUArray.h" 
#include "canlayer/handleFPUResponse.h"

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
        fpu_state.state                     = FPST_UNINITIALIZED;
        fpu_state.pending_command_set       = 0;
        for(int i=0; i < NUM_CAN_COMMANDS; i++)
        {
            fpu_state.cmd_timeouts[i]       = TimeOutList::MAX_TIMESPEC;
        }
        fpu_state.last_updated.tv_sec       = 0;
        fpu_state.last_updated.tv_nsec      = 0;
        fpu_state.timeout_count             = 0;
        fpu_state.completed_command         = CCMD_NO_COMMAND;
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

// this function returns a thread-safe copy of the current state of
// the FPU grid.  The important aspect is that the returned value is
// strictly isolated from ongoing concurrent changes in the reading
// thread.
//
// This first implementation simply copies the internal state, which
// likely returns much more information than needed.
// FIXME: replace internal type with slimmed down information
// which is actually relevant for callers.
E_GridState FPUArray::getGridState(t_grid_state& out_state)
{

#ifdef DEBUG3
    printf("copying gridstate");
#endif
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
                            t_grid_state& grid_state)
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
    pthread_mutex_unlock(&grid_state_mutex);
#ifdef DEBUG3
    printf("num_queued++ -> %i\n", FPUGridState.num_queued);
#endif    
}

int  FPUArray::countSending()
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
#ifdef DEBUG3
    printf("num_queued-- -> %i\n", FPUGridState.num_queued);
#endif    
    if ( (FPUGridState.num_queued == 0)
         && (FPUGridState.count_pending == 0) )
    {
        pthread_cond_broadcast(&cond_state_change);
        //printf("¡");fflush(stdout);
    }
    pthread_mutex_unlock(&grid_state_mutex);
    
}



E_GridState FPUArray::waitForState(E_WaitTarget target, t_grid_state& reference_state)
{

    bool got_value = false;
    E_GridState sum_state = GS_UNKNOWN;
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

    unsigned long count_timeouts = reference_state.count_timeout;

    while (! got_value)
    {

        // note this test functions *must* me be called
        // in 'locked' (mutex-protected) state!
        sum_state = getStateSummary_unprotected();
        
        // if a time-out occurs and qualifies, we return early.
        // (the counter can wrap around - no problem!)
        const bool new_timeout_triggered = (
            (count_timeouts != FPUGridState.count_timeout)
            && (target | TGT_TIMEOUT));

        // If all FPUs have been updated, that might be
        // enough.
        const bool all_updated = ((target | GS_ALL_UPDATED) &&
                             check_all_fpus_updated(num_fpus,
                                                    reference_state,
                                                    FPUGridState));


        const bool end_wait = (inTargetState(sum_state, target)
                               || new_timeout_triggered
                               || all_updated);


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
        }
        else
        {
            pthread_cond_wait(&cond_state_change,
                              &grid_state_mutex);
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

    return sum_state;
}

// this function needs to be called in locked (mutex-protected) state,
// and returns whether any state changed occured that warrants a
// return from waitForState()
bool FPUArray::inTargetState(E_GridState sum_state,
                             E_WaitTarget tstate)
{
    // if there is any unreported error
    // (such as a collision or a connection failure)
    // return true regardless of specific query.
    // FIXME: That will need some refinement for
    // error recovery (noving out of collisions etc).

    // check if the driver is working and connected -
    // if not, the state cannot change.
    E_DriverState dstate = FPUGridState.driver_state;
    if (dstate != DS_CONNECTED)
    {
        return true;
    }

    if (tstate != TGT_ANY_CHANGE)
    {
        return true;
    }

    
    // return whether we wait for no more
    // pending commands - this is needed
    // if the caller merely wants to get new
    // info from the grid, instead of a state change.
    if ((tstate | TGT_NO_MORE_PENDING)
        && (FPUGridState.count_pending == 0)
        && (FPUGridState.num_queued == 0))
    {
        return true;
    }

    
    // Now, we check whether the bit mask for
    // the state we are looking at matches
    // the return value.

    return ((sum_state | tstate) != 0);



}


bool FPUArray::isLocked(int fpu_id)
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

    // if tracing is active, signal state change
    // to waitForState() callers.
    if (num_trace_clients > 0)
    {
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


    while (true)
    {
        TimeOutList::t_toentry toentry;
        // get absolute timeout value
        //
        // FIXME: the naming is a bit confusing here.  we simply
        // define a later time to derive a value which indicates a
        // negative result (no timeout key found).
        const timespec MAX_TIMEOUT = { /* .tv_sec = */ 10,
                                       /* .tv_nsec = */ 0};

        timespec max_tmout = time_add(cur_time, MAX_TIMEOUT);
        
        next_key = tout_list.getNextTimeOut(max_tmout);
#ifdef DEBUG3
        printf("cur time: %li/%li, next timeout: %li/%li\n",
               cur_time.tv_sec, cur_time.tv_nsec,
               next_key.tv_sec, next_key.tv_nsec);
#endif
        if (time_smaller(cur_time, next_key))
        {
            break;
        }
        toentry = tout_list.pop();

        // check whether entry was found
        if (toentry.id != -1)
        {

            int fpu_id = toentry.id;

            t_fpu_state& fpu = FPUGridState.FPU_state[fpu_id];

            // remlove entries which are expired, and adjust pending count
            timespec next_timeout = expire_pending(fpu, fpu_id,
                                                   cur_time,
                                                   FPUGridState.count_pending);

            // re-insert smallest time-out of remaining pending commands
            tout_list.insertTimeOut(fpu_id, next_timeout);

            
        }

    }


    // signal any waiting control threads if
    // the grid state has changed
    if ((FPUGridState.count_pending == 0) ||
        ((old_count_pending > FPUGridState.count_pending)
         && ((num_trace_clients > 0))))
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
    FPUGridState.driver_state = dstate;
    pthread_cond_broadcast(&cond_state_change);
    pthread_mutex_unlock(&grid_state_mutex);

}


E_DriverState FPUArray::getDriverState()
{
    E_DriverState retval;

    pthread_mutex_lock(&grid_state_mutex);
    retval = FPUGridState.driver_state;
    pthread_mutex_unlock(&grid_state_mutex);
    return retval;
}


E_GridState FPUArray::getStateSummary_unprotected()
{
    // get the summary state of the grid member variable.
    // (This relies on that all FPU updates
    // do mirror the global counters correctly).

    return getGridStateSummary(FPUGridState);

}

E_GridState FPUArray::getStateSummary()
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
#ifdef DEBUG
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
            printf("RX invalid message: payload fpu_busid (%i) does"
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
#ifdef DEBUG
        printf("fpu_id too large (%i), ignored\n", fpu_id);
#endif
        return;
    }
#ifdef DEBUG3
    printf("rxFPU#%i[%i] ",fpu_id, data[1]);
    fflush(stdout);
#endif
    
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
        
        // IMPORTANT NOTE ON MESSAGE ORDERING. We can NOT assume that
        // a reply is processed /after/ the pending command was
        // registered - it can be processed before.
        //
        // In combination with that protocol version 1 does not use
        // sequence numbers, this means we can currently only have one
        // command pending at a time. If we want to send more commands
        // with a confirmed response, we always need to wait for
        // completion.
        
        FPUGridState.Counts[oldstate.state]--;
        FPUGridState.Counts[newstate.state]++;

#ifdef DEBUG3
        printf("count_pending= %i\n", FPUGridState.count_pending);
        printf("state %i -> %i\n", oldstate.state, newstate.state);
        printf("gridstate.Counts[] = [");
        for (int i=0; i < NUM_FPU_STATES; i++)
        {
            printf("%i%s", FPUGridState.Counts[i],
                   (i < (NUM_FPU_STATES -1)) ? ", " : "" );
        }
        printf("]\n"); fflush(stdout);
#endif        

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
#if 0
            int n = num_trace_clients;
            printf("![%i/%i,%i,%i]", FPUGridState.num_queued,
                   FPUGridState.count_pending,
                   state_transition ? 1 :0,
                   n);fflush(stdout);
#endif            
        }

    } // end of locked block
    int cnt =  FPUGridState.num_queued + FPUGridState.count_pending;
    pthread_mutex_unlock(&grid_state_mutex);
    
    printf("\rpending: %i    ", cnt);
    fflush(stdout);



}


timespec get_min_pending(const t_fpu_state& fpu)
{
   // get earliest previous time-out value
    timespec min_val = TimeOutList::MAX_TIMESPEC;
    for (int i=0; i < NUM_CAN_COMMANDS; i++)
    {
        if ( ((fpu.pending_command_set >> i) & 1) != 0)
        {
            timespec cmp_val = fpu.cmd_timeouts[i];
            if ( time_smaller(cmp_val, min_val))
            {
                min_val = cmp_val;
            }
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

    // add command to pending set
    fpu.pending_command_set |= ((unsigned int)1) << cmd_code;

    // get earliest previous time-out value
    timespec min_val = get_min_pending(fpu);

    fpu.cmd_timeouts[cmd_code] = new_timeout;
    // if the new value is smaller than the previous ones,
    // overwrite the time-out list entry for this fpu
    if (time_smaller(new_timeout, min_val))
    {
        timeout_list.insertTimeOut(fpu_id, new_timeout);
    }

    count_pending++;

}

void remove_pending(t_fpu_state& fpu, int fpu_id, E_CAN_COMMAND cmd_code,
                    TimeOutList& timeout_list, int &count_pending)
{
    // ignore if a command was already removed by time-out expiration


    if ( ((fpu.pending_command_set >> cmd_code) & 1) == 0)
    {
#ifdef DEBUG
    printf("fpu #%i:  cmd code %i was already removed\n", fpu_id, cmd_code);
#endif
        return;
    }

#ifdef DEBUG3
    printf("fpu #%i: removing cmd code %i %i ==> %i\n", fpu_id, cmd_code,
           fpu.pending_command_set,
           fpu.pending_command_set & ~(((unsigned int)1) << cmd_code));
#endif

    // get timespec which is to be removed

    timespec removed_val = fpu.cmd_timeouts[cmd_code];

    fpu.cmd_timeouts[cmd_code] = TimeOutList::MAX_TIMESPEC;
    
    // remove command from pending set
    fpu.pending_command_set &= ~(((unsigned int)1) << cmd_code);
    fpu.last_command = cmd_code;

    // get earliest remaining time-out value.
    // This can be MAX_TIMESPEC as well, no problem.
    timespec new_min_val = get_min_pending(fpu);

    if (time_smaller(removed_val, new_min_val))
    {
        timeout_list.insertTimeOut(fpu_id, new_min_val);
    }

    count_pending--;

}

timespec expire_pending(t_fpu_state& fpu, int fpu_id, const timespec& expiration_time,
                         int &count_pending)
{

    // remove all commands in the pending set which have a timeout
    // value before the expiration time, adjust pending count, and
    // return the remaining minimum time.

    for (int i = 0; i < NUM_CAN_COMMANDS; i++)
    {
        if ( (((fpu.pending_command_set >> i) & 1) != 0)
             && (time_smaller(fpu.cmd_timeouts[i], expiration_time)))
        {
#ifdef DEBUG
    printf("fpu #%i: expiring cmd code %i\n", fpu_id, i);
#endif
            count_pending--;
            fpu.timeout_count++;
            fpu.pending_command_set &= ~(((unsigned int)1) << i);
        }
    }
    timespec new_min_val = get_min_pending(fpu);

    return new_min_val;
}


}

}
