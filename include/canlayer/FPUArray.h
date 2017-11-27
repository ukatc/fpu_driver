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
// NAME FPUArray.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#ifndef FPU_ARRAY_H
#define FPU_ARRAY_H

#include <time.h>
#include <pthread.h>
#include <atomic>
#include <string.h>		/// memset()

#include "../E_GridState.h"
#include "../T_GridState.h"

#include "CAN_Constants.h"
#include "../DriverConstants.h"
#include "TimeOutList.h"
#include "I_CAN_Command.h"

namespace mpifps
{

namespace canlayer
{


class FPUArray
{

public:

    // maximum timeout for CAN commands which require
    // a response
    static const timespec MAX_TIMEOUT;

    typedef struct
    {
        uint8_t gateway_id;
        uint8_t bus_id;
        uint8_t can_id;
    } t_bus_address;

    typedef t_bus_address  t_bus_address_map[MAX_NUM_POSITIONERS];

    // translation table to convert FPU ids in CAN addresses.
    typedef uint16_t t_address_map[MAX_NUM_GATEWAYS][BUSES_PER_GATEWAY][FPUS_PER_BUS];



    FPUArray(int nfpus)
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

            t_fpu_state fpu_state;

            fpu_state.is_initialized       = false;
            fpu_state.state             = FPST_UNINITIALIZED;
            fpu_state.pending_command   = CCMD_NO_COMMAND;
            fpu_state.cmd_timeout       = MAX_TIMEOUT;
            fpu_state.last_updated.tv_sec = 0;
            fpu_state.last_updated.tv_nsec = 0;
            fpu_state.timeout_count        = 0;
            fpu_state.completed_command    = CCMD_NO_COMMAND;
            // the values below are not valid, they need proper
            // initialization from a physical fpu response.
            fpu_state.alpha_steps          = 0;
            fpu_state.beta_steps           = 0;
            fpu_state.on_alpha_datum       = false;
            fpu_state.on_beta_datum        = false;
            fpu_state.alpha_collision      = false;
            fpu_state.at_alpha_limit       = false;
            fpu_state.beta_collision       = false;
            fpu_state.ping_ok              = false;

            FPUGridState.FPU_state[i] = fpu_state;

        }


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
    E_GridState getGridState(t_grid_state& out_state);

    // returns summary state of FPU grid
    E_GridState getStateSummary();


    // sets and messages state changes in driver,
    // for example loss of a connection.
    void  setDriverState(E_DriverState const dstate);

    // gets state of the driver
    E_DriverState getDriverState();

    // This method waits for a certain state (actually,
    // a bitmask of states)
    // and returns the grid state when either this
    // state is reached, or when any error occurs which
    // probably requires intervention (such a collision
    // or a connection failure). It returns both
    // the detailed state of any FPU, and a summary
    // by value. Important, it must  never called by
    // the I/O threads
    // because they must not be blocked.

    E_GridState waitForState(E_WaitTarget target, t_grid_state& out_detailed_state);


    // queries whether an FPU is locked.
    // (FIXME: this is unused - delete it?)
    bool isLocked(int fpu_id);
    
    // sets pending command for one FPU.
    void setPendingCommand(int fpu_id, E_CAN_COMMAND pending_cmd, timespec tout_val);


    // sets last command for a FPU.
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
                          const int gateway_id,
                          const uint8_t busid,
                          const uint16_t canid,
                          const t_response_buf& data,
                          const int blen,
                          TimeOutList& timeOutList);


private:





///     // confirm response for one FPU, canceling the 'pending command'
///     // attributes. TODO: This should be done in the response
///     // handler.
///     void confirmResponse(int fpu_id);
/// 
    void handleFPUResponse(t_fpu_state& fpu, const t_response_buf& data,
                           const int blen);


    // returns summary state of FPU grid, without
    // lock protection.
    E_GridState getStateSummary_unprotected();


    // This internal function returns true if the
    // grid is in the requested state.
    // When this function is called,
    // the internal grid state needs to
    // be locked by the grid_state_mutex
    bool inTargetState(E_GridState sum_state,
                       E_WaitTarget tstate);

    int num_fpus;

    std::atomic<int> num_trace_clients;

    // structures which describe the current state of the whole grid
    t_grid_state FPUGridState;
    // this mutex protects the FPU state array structure
    pthread_mutex_t grid_state_mutex = PTHREAD_MUTEX_INITIALIZER;
    // condition variables which is signaled on state changes
    pthread_cond_t cond_state_change = PTHREAD_COND_INITIALIZER;

};

}

}

#endif
