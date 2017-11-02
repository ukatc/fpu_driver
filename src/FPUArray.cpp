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
FPUArray::setGridState(E_DRIVER_STATE const dstate)
{
    pthread_mutex_lock(&grid_state_mutex);
    FPUGridState.driver_state = dstate;
    pthread_cond_broadcast(&cond_state_change);
    pthread_mutex_unlock(&grid_state_mutex);

}

FPUArray::dispatchResponse(const t_address_map& fpu_id_by_adr,
                           uint8_t busid, uint16_t canid,
                           uint8_t *bytes, int blen, TimeOutList& timeOutList)
{

    // FIXME: the 16-bit canid probably not only encodes the FPU which sent
    // the response but also the response type.

    // flag to indicate the message does not address something else.

    pthread_mutex_lock(&grid_state_mutex);
    {
    
        // get canid of FPU
#pragma message "correct fpu_id computation here"
        int fpu_id = fpu_id_by_adr[gateway_id][bus_id][canid];

    
        // clear time-out flag for this FPU
        timeOutList.clearTimeOut(fpuid);
        FPUGridState.count_pending--;
    
        // TODO: unwrap response, and adjust FPU state according to
        // that


        // TODO: signal cond_state_change if a command was
        // completed (e.g. all FPUs have finished moving)
    
        t_fpu_state& fpu = FPUGridState.FPU_state[fpu_id];
        fpu.last_command = fpu.pending_command;
        fpu.pending_command = NoCommand;
    
    
        // if tracing is active, signal state change
        // to waitForState() callers.
        if (num_trace_clients > 0)
        {
            pthread_cond_broadcast(&cond_state_change);
        }

    } // end of locked block
    pthread_mutex_unlock(&grid_state_mutex);



}
