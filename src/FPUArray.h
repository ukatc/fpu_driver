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

#include <time.h>
#include <stdio.h>
#include <string.h>		/// strerror
#include <pthread.h>
//#include "stdlib.h"		/// system("/bin/stty raw");
//#include "stdbool.h"	/// bool
#include <unistd.h>
//#include <stdint.h>
#include <std>


namespace mpifps
{

typedef struct
{
    // these members are the individual values
    // reported by FPU responses
    E_FPU_STATE state;
    int alpha_steps;
    int beta_steps;
    bool is_initialized;
    bool on_alpha_datum;
    bool on_beta_datum;
    bool alpha_collision;
    bool at_alpha_limit;
    bool beta_collision;

    

    // id of any still running and incomplete command
    E_CAN_COMMAND pending_command;
    // time when any running command is considered timed out
    // NOTE THIS TIME MUST USE THE MONOTONIC LINUX SYSTEM
    // CLOCK SO THAT LEAP SECONDS DON'T TRIGGER BUGS
    time_spec cmd_timeout;
    // number of minor time-outs which have
    // been observed.
    int8 timeout_count;

    // id of last command that was completed
    E_CAN_COMMAND completed_command;

} t_fpu_state;

typedef struct
{
    // individual states of each FPU. The index
    // is always the logical ID of each FPU.
    t_FPU_state FPU_state[MAX_NUM_POSITIONERS];

    // number of collisions in current state
    int count_collision;
    // number of correctly initialised FPUs
    int count_initialised;
    // number of locked FPUs
    int count_locked;
    // number of FPUs at datum
    int count_datum;
    // number of FPUs ready to move
    int count_ready;
    // number of FPUs which are still moving
    int count_moving;
    // number of minor time-outs
    int count_timeout;
    // number of FPUs which have finished moving
    int count_finished;
    // so far unreported error
    bool at_error;
    // movement aborted
    bool at_aborted;
    // any state change
    bool state_changed;
} t_grid_state;

class FPUArray {

  public:

    // maximum timeout for CAN commands which require
    // a response
    const timespec MAX_TIMEOUT = {.tv_sec = 10,  .tv_nsec = 0};

    FPUArray()
    {

        // TODO: check if any condition variables
        // really need dynamic initialization.
        
        FPUGridState.count_collision = 0;
        FPUGridState.count_initialised = 0;
        FPUGridState.count_locked = 0;
        FPUGridState.count_ready = 0;
        FPUGridState.count_moving = 0;
        FPUGridState.count_error = 0;
        FPUGridState.count_datum = 0;
        FPUGridState.count_moving = 0;
        FPUGridState.count_timeout = 0;
        FPUGridState.count_finished = 0;

        // the last fields are flags which
        // an observer can wait for, using waitForState()
        FPUGridState.new_error = false;
        FPUGridState.new_aborted = false;
        FPUGridState.state_changed = false;

        for (int i=0; i < MAX_NUM_POSITIONERS; i++)
        {
            
            t_fpu_state fpu_state;
            
            fpu_state.is_initialized    = false;
            fpu_state.state             = UNINITIALIZED;
            fpu_state.pending_command   = NoCommand;
            fpu_state.cmd_timeout       = MAX_TIMEOUT;
            fpu_state.timeout_count     = 0;
            fpu_state.completed_command = NoCommand;
            // the values below are not valid, they need proper
            // initialization from a physical fpu response.
            fpu_state.alpha_steps       = 0;
            fpu_state.beta_steps        = 0;
            fpu_state.on_alpha_data     = false;
            fpu_state.on_beta_datum     = false;
            fpu_state.alpha_collision   = false;
            fpu_state.at_alpha_limit    = false;
            fpu_state.beta_collision    = false;

            FPUGridState.FPU_state[i] = fpu_state;
            
        }

        cached_timeout_multiplicity = 0;
        cached_timeout = MAX_TIMEOUT;
        
    }

    ~FPUArray()
    {
        // destroy condition variable
        pthread_cond_destroy(&cond_state_change);
        
        // destroy grid state mutex
        pthread_mutex_destroy(&grid_state_mutex);
    }

    // this method retrieves the current grid state for all FPUs
    // (including collision states etc). It does not wait for
    // completion of commands.  it can be called in parallel to other
    // methods.
    void  getGridState(t_grid_state& out_state);

    // this method waits for a certain state
    // and returns the grid state when either this
    // state is reached, or when any error occurs which
    // probably requires intervention (such a collision
    // or a connection failure). it returns its result
    // by value. It must  never called by the I/O threads
    // because they must not be blocked.

    void waitForState(E_WaitTarget target, t_grid_state& out_state);

    // get time of next time-out event
    // which limits the maximum waiting time for a response.
    // The returned time value is Linux' monotonic
    // clock. The max_time parameter is the
    // value which is returned if no time_out is pending.
    time_spec getNextTimeOut(time_spec max_time);

    // sets next time-out value for one FPU.
    setNextTimeOut(int fpu_id, E_CAN_COMMAND pending_command,
                   time_spec tout_val);

    // clears time-out value for a specific FPU.
    clearTimeOut(int fpu_id);


  private:


    // this function returns true if the
    // grid is in the requested state.
    // when this function is called,
    // the internal grid state needs to
    // be locked by the grid_state_mutex
    bool inTargetState(E_WaitTarget tstate);

    // flags which describe the state of the whole grid
    t_grid_state FPUGridState;
    // this mutex protects the FPU state array structure
    pthread_mutex_t grid_state_mutex = PTHREAD_MUTEX_INITIALIZER;
    // condition variables which is signaled on state changes
    pthread_cond_t cond_state_change = PTHREAD_COND_INITIALIZER;

    time_spec cached_timeout;
    int cached_timeout_multiplicity;
}

}
