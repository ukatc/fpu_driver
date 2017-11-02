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
#include "TimeOutList.h"

namespace mpifps
{

// translation table to convert FPU ids in CAN addresses.
typedef uint16 t_adress_map[MAX_NUM_GATEWAYS][BUSES_PER_GATEWAY][FPUS_PER_BUS];


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
    timespec cmd_timeout;
    // number of minor time-outs which have
    // been observed for the last command.
    int8 timeout_count;

    // id of last command that was issued but not completed.
    E_CAN_COMMAND last_command;

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
    E_DriverState driver_state;
} t_grid_state;

class FPUArray {

  public:

    // maximum timeout for CAN commands which require
    // a response
    const timespec MAX_TIMEOUT = {.tv_sec = 10,  .tv_nsec = 0};

    FPUArray(int nfpus)
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
        num_fpus = nfpus;
        num_trace_clients = 0;       
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
    // completion of commands, and can be called concurrently..
    void  getGridState(t_grid_state& out_state);


    // sets and messages state changes in driver,
    // for example loss of a connection.
    void  setGridState(E_DRIVER_STATE const dstate);
    
    // this method waits for a certain state
    // and returns the grid state when either this
    // state is reached, or when any error occurs which
    // probably requires intervention (such a collision
    // or a connection failure). it returns its result
    // by value. It must  never called by the I/O threads
    // because they must not be blocked.

    void waitForState(E_WaitTarget target, t_grid_state& out_state);

    // sets pending command for one FPU.
    
    void setPendingCommand(int fpu_id, E_CAN_COMMAND pending_cmd, timespec tout_val);

    // sets last command for a FPU

    void setLastCommand(int fpu_id, E_CAN_COMMAND last_cmd);


    
    // updates state for all FPUs which did
    // not respond in time, popping their time-out entries
    // from the list. tolist must not be locked.
    void processTimeouts(timespec cur_time, TimeOutList& tolist);

    // parses and dispatches an incoming CAN response to update the
    // state of the FPU grid. The first parameter is the mapping from
    // CAN IDs to fpu_ids. Timeouts are cleared.  Any relevant status
    // change of the grid will be signalled via the condition
    // variable.
    void dispatchResponse(const t_address_map& fpu_id_by_adr,
                          int gateway_id, uint8_t busid, uint16_t canid,
                          uint8_t *bytes, int blen, TimeOutList& timeOutList)
    
  private:


    // This internal function returns true if the
    // grid is in the requested state.
    // When this function is called,
    // the internal grid state needs to
    // be locked by the grid_state_mutex
    bool inTargetState(E_WaitTarget tstate);

    int num_fpus;

    std::atomic<int> num_trace_clients;
    
    // flags which describe the state of the whole grid
    t_grid_state FPUGridState;

    // this mutex protects the FPU state array structure
    pthread_mutex_t grid_state_mutex = PTHREAD_MUTEX_INITIALIZER;
    // condition variables which is signaled on state changes
    pthread_cond_t cond_state_change = PTHREAD_COND_INITIALIZER;

}

}
