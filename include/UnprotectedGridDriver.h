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
#include <vector>
#include "E_LogLevel.h"
#include "E_GridState.h"
#include "InterfaceConstants.h"
#include "FPUConstants.h"
#include "EtherCANInterface.h"
#include "ethercan/AsyncInterface.h"
#include "InterfaceState.h"
#include "Interval.h"


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

#define DEFAULT_START_TIMESTAMP     "ISO8601"

// UNUSED_ARG: Used for specifying that function arguments aren't used, to
// avoid build warnings
#define UNUSED_ARG(arg)     (void)arg

// Enumeration which defines the strictness of checks
// TODO: Not sure why just called "Range" - change to something more
// meaningful eventually
enum class Range
{
    Error,      // Error - path rejected
    Warn,       // Warning - path unsafe
    Ignore      // Ignore - path unchecked
};


// Define t_fpu_positions - a set of FPU alpha and beta arm positions
// TODO: t_fpu_positions is std::map<> for now because the original Python code
// in FpuGridDriver.py seems to use the corresponding data structures (e.g.
// the "hook" functions' initial_positions arguments, configuring_targets,
// configured_targets etc) as variable-sized sets of FPU positions, e.g. whose
// sizes are governed by the fpuset flags array. Would these structures
// eventually be better off as arrays of t_fpu_position's of size
// MAX_NUM_POSITIONERS?
struct t_fpu_position
{
    Interval apos;
    Interval bpos;
};
using t_fpu_positions = std::map<int, t_fpu_position>; // Keys are fpu_id's

// Forward reference for friend-ing in UnprotectedGridDriver below
class GridDriverTester;


//==============================================================================

class UnprotectedGridDriver
{
    // Declare test class as friend so that it can access protected/private
    // member variables and functions
    friend class GridDriverTester;
  
    //..........................................................................
public:
    // NOTE: Boost.Python only allows up to 14 function arguments in function
    // wrappers, so need to keep the public API functions' numbers of arguments
    // within this

    UnprotectedGridDriver(
        int nfpus = DEFAULT_NUM_FPUS,
        double SocketTimeOutSeconds = 20.0,
        bool confirm_each_step = false,
        long waveform_upload_pause_us = 0,
        int configmotion_max_retry_count = 5,
        int configmotion_max_resend_count = 10,
        int min_bus_repeat_delay_ms = 0,
        int min_fpu_repeat_delay_ms = 1,
        double alpha_datum_offset = ALPHA_DATUM_OFFSET,
        double motor_minimum_frequency = MOTOR_MIN_STEP_FREQUENCY,
        double motor_maximum_frequency = MOTOR_MAX_STEP_FREQUENCY,
        double motor_max_start_frequency = MOTOR_MAX_START_FREQUENCY,
        double motor_max_rel_increase = MAX_ACCELERATION_FACTOR,
        double motor_max_step_difference = MAX_STEP_DIFFERENCE
        );

    // TODO: Check if this virtual destructor stuff is correct
    // TODO: Need a real destructor as well?? Or are all member objects RAII ones?
    virtual ~UnprotectedGridDriver();

    E_EtherCANErrCode initialize(
        E_LogLevel logLevel = DEFAULT_LOGLEVEL,
        const std::string &log_dir = DEFAULT_LOGDIR,
        int firmware_version_address_offset = 0x61,
        const std::string &protection_logfile = "_" DEFAULT_START_TIMESTAMP "-fpu_protection.log",
        const std::string &control_logfile = "_" DEFAULT_START_TIMESTAMP "-fpu_control.log",
        const std::string &tx_logfile = "_" DEFAULT_START_TIMESTAMP "-fpu_tx.log",
        const std::string &rx_logfile = "_" DEFAULT_START_TIMESTAMP "-fpu_rx.log",
        const std::string &start_timestamp = DEFAULT_START_TIMESTAMP);

    virtual bool initializedOk();

    E_GridState getGridState(t_grid_state &grid_state_ret);

    E_EtherCANErrCode connect(int ngateways,
                              const t_gateway_address gateway_addresses[]);
    E_EtherCANErrCode disconnect();

    E_EtherCANErrCode findDatum(t_grid_state &gs,
                        const t_datum_search_flags &search_modes,
                        enum E_DATUM_SELECTION selected_arm,
                        const t_fpuset &fpuset,
                        bool soft_protection = true,
                        bool count_protection = true,
                        bool support_uninitialized_auto = true,
                        enum E_DATUM_TIMEOUT_FLAG timeout = DATUM_TIMEOUT_ENABLE);

    virtual E_EtherCANErrCode pingFPUs(t_grid_state &gs, const t_fpuset &fpuset);

    E_EtherCANErrCode resetFPUs(t_grid_state &gs, const t_fpuset &fpuset);

    E_EtherCANErrCode readSerialNumbers(t_grid_state &gs, const t_fpuset &fpuset);

    E_EtherCANErrCode configMotion(const t_wtable &wavetable, t_grid_state &gs,
                        const t_fpuset &fpuset,
                        bool soft_protection = true,
                        bool allow_uninitialized = false,
                        int ruleset_version = DEFAULT_WAVEFORM_RULESET_VERSION,
                        bool warn_unsafe = true,
                        int verbosity = 3);

    E_EtherCANErrCode executeMotion(t_grid_state &gs, const t_fpuset &fpuset,
                                    bool sync_command = true);
                                
    E_EtherCANErrCode enableMove(int fpu_id, t_grid_state &gs);

    //..........................................................................
protected:
    // NOTE: The following virtual functions are overridden in GridDriver

    virtual void _post_connect_hook() {}

    // findDatum() hook functions
    // TODO: Do the t_grid_state's below need to be const? Or will they
    // possibly be altered inside the functions?
    virtual void _allow_find_datum_hook(t_grid_state &gs,
                                        t_datum_search_flags &search_modes, // Modifiable
                                        enum E_DATUM_SELECTION selected_arm,
                                        const t_fpuset &fpuset,
                                        bool support_uninitialized_auto = true)
    {
        UNUSED_ARG(gs);
        UNUSED_ARG(search_modes);
        UNUSED_ARG(selected_arm);
        UNUSED_ARG(fpuset);
        UNUSED_ARG(support_uninitialized_auto);
    }
    virtual void _start_find_datum_hook(t_grid_state &gs,
                                        const t_datum_search_flags &search_modes,
                                        enum E_DATUM_SELECTION selected_arm,
                                        const t_fpuset &fpuset,
                                        t_fpu_positions &initial_positions_ret,
                                        bool soft_protection)
    {
        UNUSED_ARG(gs);
        UNUSED_ARG(search_modes);
        UNUSED_ARG(selected_arm);
        UNUSED_ARG(fpuset);
        UNUSED_ARG(initial_positions_ret);
        UNUSED_ARG(soft_protection);
    }
    virtual void _cancel_find_datum_hook(t_grid_state &gs,
                                         // TODO: These arguments (which are in the Python version)
                                         // are not used so can remove?
                                         //const t_datum_search_flags &search_modes,
                                         //enum E_DATUM_SELECTION selected_arm,
                                         const t_fpuset &fpuset, 
                                         const t_fpu_positions &initial_positions)
    {
        UNUSED_ARG(gs);
        UNUSED_ARG(fpuset);
        UNUSED_ARG(initial_positions);
    }
    virtual void _finished_find_datum_hook(t_grid_state &prev_gs,
                                           t_grid_state &datum_gs,
                                           const t_datum_search_flags &search_modes,
                                           const t_fpuset &fpuset,
                                           bool was_cancelled,
                                           const t_fpu_positions &initial_positions,
                                           enum E_DATUM_SELECTION selected_arm)
    {
        UNUSED_ARG(prev_gs);
        UNUSED_ARG(datum_gs);
        UNUSED_ARG(search_modes);
        UNUSED_ARG(fpuset);
        UNUSED_ARG(was_cancelled);
        UNUSED_ARG(initial_positions);
        UNUSED_ARG(selected_arm);
    }

    // resetFPUs() hook function
    virtual void _reset_hook(t_grid_state &old_state, t_grid_state &gs,
                             const t_fpuset &fpuset)
    {
        UNUSED_ARG(old_state);
        UNUSED_ARG(gs);
        UNUSED_ARG(fpuset);
    }

    // Error counters function
    virtual void _update_error_counters(const t_fpu_state &prev_fpu,
                                        const t_fpu_state &moved_fpu,
                                        bool datum_cmd = false)
    {
        UNUSED_ARG(prev_fpu);
        UNUSED_ARG(moved_fpu);
        UNUSED_ARG(datum_cmd);
    }

    // configMotion() hook functions
    virtual void _pre_config_motion_hook(const t_wtable &wtable,
                                         t_grid_state &gs,
                                         const t_fpuset &fpuset,
                                         Range wmode = Range::Error)
    {
        UNUSED_ARG(wtable);
        UNUSED_ARG(gs);
        UNUSED_ARG(fpuset);
        UNUSED_ARG(wmode);
    }
    virtual void _post_config_motion_hook(const t_wtable &wtable,
                                          t_grid_state &gs,
                                          const t_fpuset &fpuset)
    {
        // NOTE: The original Python version of this function called
        // set_wtable_reversed(), but this isn't required because it doesn't
        // do anything in UnprotectedGridDriver (because wf_reversed isn't
        // used in it)
        UNUSED_ARG(wtable);
        UNUSED_ARG(gs);
        UNUSED_ARG(fpuset);
    }

    // repeatMotion() hook functions
    virtual void _pre_repeat_motion_hook(const t_wtable &wtable,
                                         t_grid_state &gs,
                                         const t_fpuset &fpuset,
                                         Range wmode = Range::Error)
    {
        UNUSED_ARG(wtable);
        UNUSED_ARG(gs);
        UNUSED_ARG(fpuset);
        UNUSED_ARG(wmode);
    }
    virtual void _post_repeat_motion_hook(const t_wtable &wtable,
                                          t_grid_state &gs,
                                          const t_fpuset &fpuset)
    {
        // NOTE: See comments in _post_config_motion_hook() above - they apply
        // to this function as well
        UNUSED_ARG(wtable);
        UNUSED_ARG(gs);
        UNUSED_ARG(fpuset);
    }

    // reverseMotion() hook functions
    virtual void _pre_reverse_motion_hook(const t_wtable &wtable,
                                          t_grid_state &gs,
                                          const t_fpuset &fpuset,
                                          Range wmode = Range::Error)
    {
        UNUSED_ARG(wtable);
        UNUSED_ARG(gs);
        UNUSED_ARG(fpuset);
        UNUSED_ARG(wmode);
    }  
    virtual void _post_reverse_motion_hook(const t_wtable &wtable,
                                          t_grid_state &gs,
                                          const t_fpuset &fpuset)
    {
        // NOTE: See comments in _post_config_motion_hook() above - they apply
        // to this function as well
        UNUSED_ARG(wtable);
        UNUSED_ARG(gs);
        UNUSED_ARG(fpuset);
    }

    // executeMotion() hook functions
    virtual void _start_execute_motion_hook(t_grid_state &gs,
                                            const t_fpuset &fpuset,
                                            const t_fpu_positions &initial_positions)
    {
        UNUSED_ARG(gs);
        UNUSED_ARG(fpuset);
        UNUSED_ARG(initial_positions);
    }
    virtual void _cancel_execute_motion_hook(t_grid_state &gs,
                                             const t_fpuset &fpuset,
                                             const t_fpu_positions &initial_positions)
    {
        UNUSED_ARG(gs);
        UNUSED_ARG(fpuset);
        UNUSED_ARG(initial_positions);
    }
    virtual void _post_execute_motion_hook(t_grid_state &gs,
                                           const t_grid_state &old_gs,
                                           const t_grid_state &move_gs,
                                           const t_fpuset &fpuset)
    {
        UNUSED_ARG(gs);
        UNUSED_ARG(old_gs);
        UNUSED_ARG(move_gs);
        UNUSED_ARG(fpuset);
    }

    E_EtherCANErrCode check_fpuset(const t_fpuset &fpuset);
    void need_ping(const t_grid_state &gs, const t_fpuset &fpuset,
                   t_fpuset &pingset_ret);
    E_EtherCANErrCode _pingFPUs(t_grid_state &gs, const t_fpuset &fpuset);
    // N.B. static function
    static bool wavetable_was_received(const t_wtable &wtable, int fpu_id,
                                       const t_fpu_state &fpu_state,  
                                       bool allow_unconfirmed = false,
                                       E_FPU_STATE target_state = FPST_READY_FORWARD);
    void sleepSecs(double seconds);

    // Config: Only the constructor and initialize() must write this - do NOT
    // write at all after initialize() has been called
    EtherCANInterfaceConfig config;
    bool initialize_was_called_ok = false;

    // TODO: See comments above t_wtable definition in AsyncInterface.h
    t_wtable last_wavetable;

    // NOTE: wf_reversed is not used in UnprotectedGridDriver, so it has been
    // moved to GridDriver (N.B. The original Python version had it in
    // UnprotectedGridDriver)
    // std::vector<bool> wf_reversed;

    // _gd: N.B. Can't use a unique_ptr for EtherCANInterface, probably
    // because it's explicitly non-movable
    // TODO: Rename to something like etherCanIfPtr eventually - it's named as
    // _gd so that it's the same as in FpuGridDriver.py for now
    EtherCANInterface *_gd = nullptr;

    //..........................................................................
private:
    E_EtherCANErrCode pingIfNeeded(t_grid_state &gs, const t_fpuset &fpuset);

    bool wavetables_incomplete = false;

    // TODO: Add locked_gateways here, but need to convert Python devicelock.py
    // to C++ first (and use a Linux named semaphore instead of a lock file for
    // this?)

    //..........................................................................
};


//==============================================================================

} // namespace mpifps

#endif // UNPROTECTEDGRIDDRIVER_H
