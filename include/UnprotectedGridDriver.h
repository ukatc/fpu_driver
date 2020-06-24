// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-04-28  TODO: Put comment here
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME UnprotectedGridDriver.h
//
// TODO: Put description here
//
////////////////////////////////////////////////////////////////////////////////

#ifndef UNPROTECTEDGRIDDRIVER_H
#define UNPROTECTEDGRIDDRIVER_H

#include <string>
#include <map>
#include <memory>
#include "E_LogLevel.h"
#include "InterfaceConstants.h"
#include "FPUConstants.h"
#include "EtherCANInterface.h"
#include "AsyncInterface.h"
#include "InterfaceState.h"
#include "Interval.h"

// TODO: FOR TESTING ONLY
#include "ProtectionDB.h"

namespace mpifps
{

using namespace std;
using namespace mpifps::ethercanif;

// *** TODO: Temporary dummy values for now - need to get from the Linux 
// environment variables in same way as is done in FpuGridDriver.py - need to
// figure out the C/C++ equivalents 
#define DEFAULT_NUM_FPUS    (1)
#define DEFAULT_LOGLEVEL    (LOG_ERROR) 
#define DEFAULT_LOGDIR      ("$HOME")

//*************** TODO: Await Steven's OK for FPU set being a std::vector<>
// (rather than the t_fpuset array of bools)
// BUT use t_fpuset from EtherCAN after all?
//#define FPU_SET_IS_VECTOR

#ifdef FPU_SET_IS_VECTOR
// TODO: This is named "FpuSelection" to clearly differentiate its type from
// the EtherCAN layer's t_fpuset type
using FpuSelection = std::vector<uint16_t>;
#endif

// Enumeration which defines the strictness of checks
// TODO: Not sure why just called "Range" - change to something more
// meaningful eventually
enum class Range
{
    Error,      // Error - path rejected
    Warn,       // Warning - path unsafe
    Ignore      // Ignore - path unchecked
};

// TODO: Not sure about whether these FPU position data structures are fully
// correct and suitable yet - keep under review
typedef struct
{
    Interval apos;
    Interval bpos;
} FpuPosition;

typedef FpuPosition FpuPositions[MAX_NUM_POSITIONERS];

// Forward reference for friend class declaration in UnprotectedGridDriver below
class UnprotectedGridDriverTester;


//==============================================================================

class UnprotectedGridDriver
{
    // Declare test class as friend so that it can access protected/private
    // member variables and functions
    friend class UnprotectedGridDriverTester;
  
    //..........................................................................
public:
    UnprotectedGridDriver(
        // NOTE: Boost.Python only allows up to 14 function arguments in
        // function wrappers, so need to keep number of arguments within this
        int nfpus = DEFAULT_NUM_FPUS,
        bool confirm_each_step = false,
        int configmotion_max_retry_count = 5,
        int configmotion_max_resend_count = 10,
        int min_bus_repeat_delay_ms = 0,
        int min_fpu_repeat_delay_ms = 1,
        enum E_LogLevel logLevel = DEFAULT_LOGLEVEL,
        const string &log_dir = DEFAULT_LOGDIR,
        double motor_minimum_frequency = MOTOR_MIN_STEP_FREQUENCY,
        double motor_maximum_frequency = MOTOR_MAX_STEP_FREQUENCY,
        double motor_max_start_frequency = MOTOR_MAX_START_FREQUENCY,
        double motor_max_rel_increase = MAX_ACCELERATION_FACTOR
        );

    E_EtherCANErrCode connect(const int ngateways,
                              const t_gateway_address gateway_addresses[]);

#ifndef FPU_SET_IS_VECTOR
    // TODO: In some of the following functions, reversed the 
    // fpuset/selected_arm arguments relative to the equivalent Python
    // functions, so that can have argument defaults

    E_EtherCANErrCode findDatum(t_grid_state &gs,
                                const t_datum_search_flags &search_modes,
                                enum E_DATUM_SELECTION selected_arm,
                                const t_fpuset &fpuset,
                                bool soft_protection, bool count_protection,
                                bool support_uninitialized_auto,
                                enum E_DATUM_TIMEOUT_FLAG timeout);

    E_EtherCANErrCode configMotion(const t_wtable &wavetable, t_grid_state &gs,
                                   const t_fpuset &fpuset, bool soft_protection,
                                   bool allow_uninitialized,
                                   int ruleset_version, bool warn_unsafe,
                                   int verbosity);

    E_EtherCANErrCode executeMotion(t_grid_state &gs, const t_fpuset &fpuset,
                                    bool sync_command = true);

#endif // FPU_SET_IS_VECTOR

    // TODO: Check if this virtual destructor stuff is correct
    // TODO: Need a real destructor as well?? Or are all member objects RAII ones?
    virtual ~UnprotectedGridDriver();

    //..........................................................................
protected:
    // NOTE: The following virtual functions are overridden in GridDriver

    virtual void _post_connect_hook(const EtherCANInterfaceConfig &config) {}

    // findDatum() hook functions
    // TODO: Do the t_grid_state's below need to be const? Or will they
    // possibly be altered inside the functions?
    virtual void _allow_find_datum_hook(t_grid_state &gs,
                                        t_datum_search_flags &search_modes, // Modifiable
                                        enum E_DATUM_SELECTION selected_arm,
                                        const t_fpuset &fpuset,
                                        bool support_uninitialized_auto) {}
    virtual void _start_find_datum_hook(t_grid_state &gs,
                                        const t_datum_search_flags &search_modes,
                                        enum E_DATUM_SELECTION selected_arm,
                                        const t_fpuset &fpuset,
                                        FpuPositions &initial_positions_ret,
                                        bool soft_protection) {}
    virtual void _cancel_find_datum_hook(t_grid_state &gs,
                                         // TODO: These arguments (which are in the Python version)
                                         // are not used so can remove?
                                         //const t_datum_search_flags &search_modes,
                                         //enum E_DATUM_SELECTION selected_arm,
                                         const t_fpuset &fpuset, 
                                         const FpuPositions &initial_positions) {}
    virtual void _finished_find_datum_hook(t_grid_state &prev_gs,
                                           t_grid_state &datum_gs,
                                           const t_datum_search_flags &search_modes,
                                           const t_fpuset &fpuset,
                                           bool was_cancelled,
                                           const FpuPositions &initial_positions,
                                           enum E_DATUM_SELECTION selected_arm) {}

    // Error counters function
    virtual void _update_error_counters(const t_fpu_state &prev_fpu,
                                        const t_fpu_state &moved_fpu,
                                        bool datum_cmd = false) {}

    // configMotion() hook functions
    virtual void _pre_config_motion_hook(const t_wtable &wtable,
                                         t_grid_state &gs,
                                         const t_fpuset &fpuset, Range wmode) {}
    virtual void _post_config_motion_hook(const t_wtable &wtable,
                                          t_grid_state &gs,
                                          const t_fpuset &fpuset)
    {
        // TODO: Add C++/Linux equivalent of Python version's "with self.lock"
        // here

        set_wtable_reversed(fpuset, false);
    }

    // executeMotion() hook functions
    virtual void _start_execute_motion_hook(t_grid_state &gs,
                                            const t_fpuset &fpuset,
                                            const FpuPositions &initial_positions) {}
    virtual void _cancel_execute_motion_hook(t_grid_state &gs,
                                             const t_fpuset &fpuset,
                                             const FpuPositions &initial_positions) {}
    virtual void _post_execute_motion_hook(t_grid_state &gs,
                                           const t_grid_state &old_gs,
                                           const t_grid_state &move_gs,
                                           const t_fpuset &fpuset) {}

    EtherCANInterfaceConfig config;

    //..........................................................................
private:
  
#ifdef FPU_SET_IS_VECTOR
    E_EtherCANErrCode check_fpuset(const FpuSelection &fpu_selection);
    void need_ping(const t_grid_state &gs,
                   const FpuSelection &fpu_selection,
                   FpuSelection &fpu_ping_selection_ret);
#else // NOT FPU_SET_IS_VECTOR
    E_EtherCANErrCode check_fpuset(const t_fpuset &fpuset);
    void need_ping(const t_grid_state &gs, const t_fpuset &fpuset,
                   t_fpuset &pingset_ret);
#endif // NOT FPU_SET_IS_VECTOR
    E_EtherCANErrCode _pingFPUs(t_grid_state &gs, const t_fpuset &fpuset);
    // N.B. static function
    static bool wavetable_was_received(const t_wtable &wtable, int fpu_id,
                                       const t_fpu_state &fpu_state,  
                                       bool allow_unconfirmed = false,
                                       E_FPU_STATE target_state = FPST_READY_FORWARD);
    void set_wtable_reversed(const t_fpuset &fpuset, bool is_reversed);

    // TODO: Use fixed-size array of MAX_NUM_POSITIONERS like the t_fpuset
    // structures etc do?
    t_wtable last_wavetable;

    // TODO: Use fixed-size array of MAX_NUM_POSITIONERS like the t_fpuset
    // structures etc do?
    std::map<int, bool> wf_reversed; // <fpu_id, reversed>

    bool wavetables_incomplete = false;

    // _gd: N.B. Can't use a unique_ptr for EtherCANInterface, probably
    // because it's explicitly non-movable
    // TODO: Rename to something like etherCanIfPtr eventually - is  _gd so that
    // it's same as in FpuGridDriver.py for now
    EtherCANInterface *_gd = nullptr;

    // TODO: Add locked_gateways here, but need to convert Python devicelock.py
    // to C++ first (and use a Linux named semaphore instead of a lock file for this?)

    //..........................................................................
};


//==============================================================================

class UnprotectedGridDriverTester
{
    // N.B. This class is friend-ed from UnprotectedGridDriver, so it can
    // access its private and protected member variables and functions
public:
    void doTests();
    
private:
    void test_check_fpuset();
    void test_need_ping();
    void test_connect();
    void test_findDatum();
};


//==============================================================================

} // namespace mpifps

#endif // UNPROTECTEDGRIDDRIVER_H
