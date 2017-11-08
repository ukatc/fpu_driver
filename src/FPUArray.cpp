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


#include <stdio.h>
#include <string.h>		/// strerror
#include <pthread.h>
#include <unistd.h>
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
E_GridState FPUarray::getGridState(t_grid_state& out_state)
{

    pthread_mutex_lock(&grid_state_mutex);
    // we simply copy the internal state
    out_state = FPUGridState;
    pthread_mutex_unlock(&grid_state_mutex);
    E_GridState = getGridStateSummary(out_state);
}


E_GridState FPUarray::waitForState(E_WaitTarget target, t_grid_state& out_detailed_state)
{

    bool got_value = false;
    R_GridState sum_state = GS_UNKNOWN;
    // if we want to get signaled on any minor changes,
    // we increment a special counter to trigger
    // additional event notifications.
    if (target == ANY_CHANGE)
    {
        num_trace_clients++;
    }
    pthread_mutex_lock(&grid_state_mutex);

    unsigned long count_timeouts = FPUGridState.count_timeout;

    while (! got_value)
    {

        // note this test functions *must* me be called
        // in 'locked' (mutex-protected) state!
        sum_state = getStateSummary_unprotected();
        // if a time-out occurs, we return always.
        // (the counter can wrap around - no problem!)
        bool timeout_count_unchanged = (
            count_timeouts == FPUGridState.count_timeout);
        
        if ((! inTargetState(sum_state, target)
             && timeout_count_unchanged)
        {
            pthread_cond_wait(&cond_state_change,
                              &grid_state_mutex);
        }
        else
        {
            // We copy the internal state.  This is a comperatively
            // expensive operation which as of today (2017) has a
            // latency of about 25 microseconds. The reason we
            // make a copy here is that it can't change later, therefore
            // reading the copy does not need any locking.
            out_detailed_state = FPUGridState;
            got_value = true;
        }
    }
    pthread_mutex_unlock(&grid_state_mutex);
    if (target == ANY_CHANGE)
    {
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

    // check if the driver is working -
    // otherwise, the state cannot change.
    E_DriverState dstate = grid_state.driver_state;
    if (dstate != CONNECTED)
    {
        return true;
    }

    if (tstate != TGT_ANY_CHANGE)
    {
        return true;
    }

    // Now, we check whether the bit mask for
    // the state we are looking at matches
    // the return value.

    return ((sum_state | tstate) != 0);
    

            
    }
    
}


// sets pending command for one FPU.
    
void FPUArray::setPendingCommand(int fpu_id, E_CAN_COMMAND pending_cmd, timespec tout_val)
{
    pthread_mutex_lock(&grid_state_mutex);

    FPUGridState.count_pending++;

    t_fpu_state& fpu = FPUGridState.FPU_state[fpu_id];
    fpu.pending_command = pending_cmd;

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

    FPUGridState.count_pending++;

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


void FPUArray::confirmCommand(int fpu_id)
{
    
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


void FPUArray::processTimeouts(timespec cur_time, TimeOutList& tolist)
{
    bool new_timeout = false;
    timespec next_key;
    pthread_mutex_lock(&grid_state_mutex);

    while (true)
    {
        t_toentry to_entry;
        next_key = timeOutList.getNextTimeOut(MAX_TIMEOUT);
        if (time_smaller(cur_time, next_key))
        {
            break;
        }
        to_entry = timeOutList.pop();

        int fpu_id = to_entry.id;
        new_timeout = true;
        FPUGridState.count_timeout++;
        FPUGridState.count_pending--;

        t_fpu_state& fpu = FPUGridState.FPU_state[fpu_id];
        fpu.last_command = fpu_pending_command;
        fpu.pending_command = NoCommand;
        fpu.timeout_count++;
        
        
        
    }

    // signal any waiting control threads that
    // the grid state has changed
    if (new_timeout)
    {
        pthread_cond_broadcast(&cond_state_change);
    }
    pthread_mutex_unlock(&grid_state_mutex);
    
};


// This function sets the global state of
// the CAN driver. It allows to notify
// callers of waitForState() when any relevant
// change of the system happens, such as a lost
// socket connection.
void FPUArray::setDriverState(E_DRIVER_STATE const dstate)
{
    pthread_mutex_lock(&grid_state_mutex);
    FPUGridState.driver_state = dstate;
    pthread_cond_broadcast(&cond_state_change);
    pthread_mutex_unlock(&grid_state_mutex);

}


E_DRIVER_STATE FPUArray::getDriverState()
{
    E_DRIVER_STATE retval;
    
    pthread_mutex_lock(&grid_state_mutex);
    retsval = FPUGridState.driver_state;
    pthread_mutex_unlock(&grid_state_mutex);
}


E_GridState FPUArray::getStateSummary_unprotected()
{
    // get the summary state of the grid member variable.
    // (This relies on that all FPU updates
    // do mirror the global counters correctly).

    return getGridStateSummary(FPUGrid);
        
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
                                int gateway_id, uint8_t busid,
                                uint16_t can_identifier,
                                uint8_t& data[8], int blen,
                                TimeOutList& timeOutList)
{

    // FIXME: the 16-bit canid probably not only encodes the FPU which sent
    // the response but also the response type.

    // flag to indicate the message does not address something else.

    pthread_mutex_lock(&grid_state_mutex);
    {
    
        // get canid of FPU (additional ids might be used
        // to report state of the gateway)
        
#pragma message "insert correct fpu_id computation here"
        // let's assume the CAN identifier has a response type
        // code in bits 7 to 10, and the FPU id in bits
        //

        uint cmd_id = (can_identifier >> 7);
        int fpu_id = fpu_id_by_adr[gateway_id][bus_id][can_identifier & 128];

    
        // clear time-out flag for this FPU
        timeOutList.clearTimeOut(fpuid);
        FPUGridState.count_pending--;

        // FIXME: we need to adjust the time-out count
    
        // TODO: unwrap response, and adjust FPU state according to
        // that


        // TODO: signal cond_state_change if a command was
        // completed (e.g. all FPUs have finished moving)

        handleFPUResponse(FPUGridState.FPU_state[fpu_id], data);    
    
        // if no more commands are pending or tracing is active,
        // signal a state change to waitForState() callers.
        if ((FPUGridState.count_pending == 0)
            || (num_trace_clients > 0) )
        {
            pthread_cond_broadcast(&cond_state_change);
        }

    } // end of locked block
    pthread_mutex_unlock(&grid_state_mutex);



}


void FPUArray::handleFPUResponse(t_fpu_state& fpu, uint8_t& data[8],
                                 int blen)
{
    fpu.last_command = fpu.pending_command;
    switch (cmd_id)
    {
    case PING_RESPONSE :
        // that's just placeholder code
        fpu.ping_ok = true;
        break;
        // TODO: INSERT CORRESPONDING STATE CHANGES HERE
    }
    fpu.pending_command = NoCommand;

}
