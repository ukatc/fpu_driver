// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
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
#include "../InterfaceConstants.h"
#include "../EtherCANInterfaceConfig.h"
#include "TimeOutList.h"
#include "CAN_Command.h"

/* Switches on use of monotonic clock for timed waits
   on grid state changes. This is advisable to avoid
   nasty bugs on changes of system time, but currently
   causes problems when de-initialising the condition
   variable. The clocks used for epoll() are not
   affected. */
#define FPUARRAY_USE_MONOTONIC_CLOCK 0

namespace mpifps
{

namespace ethercanif
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
    // for the can bus ID, the index 0 is not used
    typedef uint16_t t_address_map[MAX_NUM_GATEWAYS][BUSES_PER_GATEWAY][1 + FPUS_PER_BUS];


    explicit FPUArray(const EtherCANInterfaceConfig &config_vals);

    ~FPUArray();

    // initialize internal structures
    E_EtherCANErrCode initialize();

    // deinitialize internal structures
    E_EtherCANErrCode deInitialize();

    // this method retrieves the current grid state for all FPUs
    // (including collision states etc). It does not wait for
    // completion of commands, and can be called concurrently..
    E_GridState getGridState(t_grid_state& out_state) const;

    // returns summary state of FPU grid
    E_GridState getStateSummary() const;


    // sets and messages state changes in driver,
    // for example loss of a connection.
    void  setInterfaceState(E_InterfaceState const dstate);

    // gets state of the driver
    E_InterfaceState getInterfaceState() const;

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
    //
    // Optionally, a maximum wait time can be passed after which the
    // method will always return.  If the waiting operation was
    // cancelled due to exceeding that maximum time, the reference
    // paremeter cancelled is set to true, otherwise to false.  A
    // maximum time of zero means no time limit - the caller has to
    // make sure that the condition waited for can be met.

    E_GridState waitForState(E_WaitTarget target, t_grid_state& out_detailed_state,
                             double &max_wait_time, bool &cancelled) const;


    // queries whether an FPU is locked.
    // (FIXME: this is unused - delete it?)
    bool isLocked(int fpu_id) const;

    // sets pending command for one FPU.
    void setPendingCommand(int fpu_id, E_CAN_COMMAND pending_cmd, timespec tout_val,
                           uint8_t sequence_number, TimeOutList& timeout_list);


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


    // this field can be updated by the tx thread without extra
    // locking - it is used for commands which are
    // being prepared but which might not yet appear
    // as "pending".


    // increment and decrement number of commands
    // which are currently sent.
    void incSending();
    void decSending();

    // increments and fetches the next message sequence number
    // for this FPU
    uint8_t countSequenceNumber(const int fpu_id, const bool increment, const bool broadcast);

    // get number of commands which are being sent.
    int  countSending() const;

private:





///     // confirm response for one FPU, canceling the 'pending command'
///     // attributes.

///    void handleFPUResponse(int fpu_id, t_fpu_state& fpu, const t_response_buf& data,
///                           const int blen);
///


    // returns summary state of FPU grid, without
    // lock protection.
    E_GridState getStateSummary_unprotected() const;


    // This internal function returns true if the
    // grid is in the requested state.
    // When this function is called,
    // the internal grid state needs to
    // be locked by the grid_state_mutex
    bool inTargetState(E_GridState sum_state,
                       E_WaitTarget tstate) const;

    const EtherCANInterfaceConfig config;

    mutable std::atomic<int> num_trace_clients;

    // structures which describe the current state of the whole grid
    t_grid_state FPUGridState;
    // this mutex protects the FPU state array structure
    mutable pthread_mutex_t grid_state_mutex = PTHREAD_MUTEX_INITIALIZER;
    // condition variables which is signaled on state changes
#if FPUARRAY_USE_MONOTONIC_CLOCK
    mutable pthread_cond_t cond_state_change; // is initialized with monotonic clock option
#else
    mutable pthread_cond_t cond_state_change = PTHREAD_COND_INITIALIZER;
#endif
};


// add new pending command to pending command set and time-out list
void add_pending(t_fpu_state& fpu, int fpu_id, E_CAN_COMMAND cmd_code,
                 const timespec& new_timeout,
                 TimeOutList& timeout_list,
                 int &count_pending,
                 const uint8_t sequence_number);

// remove a command from the pending command set, and refresh the
// time-out list with the next time out.
void remove_pending(const EtherCANInterfaceConfig &config,
                    t_fpu_state& fpu, int fpu_id,
                    E_CAN_COMMAND cmd_code, E_MOC_ERRCODE cmd_status,
                    TimeOutList& timeout_list,
                    int &count_pending,
		    uint8_t msg_sequence_number);

// Remove time out entries which are earlier than the expiration time
// from the fpu pending set, and return the next time-out value from
// the remaining set (or MAX_TIMESPEC if the set is empty)
timespec expire_pending(const EtherCANInterfaceConfig &config,
                        t_fpu_state& fpu, int fpu_id, const timespec& expiration_time,
                        int  &count_pending, unsigned long &count_timeouts);



}

}

#endif
