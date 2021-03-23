// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-04-28  Created (translated from Python FpuGridDriver.py).
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME UnprotectedGridDriver.h
//
// This UnprotectedGridDriver class provides the main higher-level unprotected
// functionality for the grid driver. It also provides virtual, mostly-empty
// "hook" functions which are called from various places. The separate
// GridDriver class inherits from this UnprotectedGridDriver class and provides
// the FPU movement software protection functionality in its overrides of the
// hook functions.
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
#include "FPUCommands.h"
#include "FPUConstants.h"
#include "EtherCANInterface.h"
#include "ethercan/AsyncInterface.h"
#include "InterfaceState.h"
#include "ErrorCodes.h"
#include "Interval.h"
// N.B. The UnprotectedGridDriver class doesn't use the protection database,
// but it does use the FpuDbData structure defined in ProtectionDB.h - see
// comments above the FpuData structure definition elsewhere in this class.
#include "ProtectionDB.h" 

namespace mpifps
{

using namespace std;
using namespace mpifps::ethercanif;

//..............................................................................

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
// eventually be better off as std::vector's or arrays of t_fpu_position's of
// size MAX_NUM_POSITIONERS or num_fpus?
struct t_fpu_position
{
    Interval apos;
    Interval bpos;
};
using t_fpu_positions = std::map<int, t_fpu_position>; // Keys are fpu_id's


//==============================================================================

void gridDriverAbortDuringFindDatumOrExecuteMotion(void);

// Forward reference for friend-ing in UnprotectedGridDriver below
class GridDriverTester;

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
#ifndef FLEXIBLE_CAN_MAPPING // NOT FLEXIBLE_CAN_MAPPING
        int nfpus = DEFAULT_NUM_FPUS,
#endif // NOT FLEXIBLE_CAN_MAPPING
        double SocketTimeOutSeconds = SOCKET_TIMEOUT_SECS,
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
#ifdef FLEXIBLE_CAN_MAPPING
        const std::string &can_map_file_path,
#endif // FLEXIBLE_CAN_MAPPING
        E_LogLevel logLevel = DEFAULT_LOGLEVEL,
        const std::string &log_dir = DEFAULT_LOGDIR,
        int firmware_version_address_offset = 0x61,
        const std::string &protection_logfile = "_" DEFAULT_START_TIMESTAMP "-fpu_protection.log",
        const std::string &control_logfile = "_" DEFAULT_START_TIMESTAMP "-fpu_control.log",
        const std::string &tx_logfile = "_" DEFAULT_START_TIMESTAMP "-fpu_tx.log",
        const std::string &rx_logfile = "_" DEFAULT_START_TIMESTAMP "-fpu_rx.log",
        const std::string &start_timestamp = DEFAULT_START_TIMESTAMP);

    virtual bool initializedOk();

    E_EtherCANErrCode connect(int ngateways,
                              const t_gateway_address gateway_addresses[]);
    E_EtherCANErrCode disconnect();

    E_EtherCANErrCode setUStepLevel(int ustep_level, t_grid_state &gs,
                                    const t_fpuset &fpuset);
    E_EtherCANErrCode setTicksPerSegment(unsigned long nticks,
                                         t_grid_state &gs,
                                         const t_fpuset &fpuset);
    E_EtherCANErrCode setStepsPerSegment(int min_steps, int max_steps,
                                         t_grid_state &gs,
                                         const t_fpuset &fpuset);

    E_GridState getGridState(t_grid_state &grid_state_ret);

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
    E_EtherCANErrCode resetStepCounters(long new_alpha_steps, long new_beta_steps,
                                        t_grid_state &gs, const t_fpuset &fpuset);

    E_EtherCANErrCode readRegister(uint16_t address, t_grid_state &gs,
                                   const t_fpuset &fpuset);
    E_EtherCANErrCode  getDiagnostics(t_grid_state &gs, const t_fpuset &fpuset,
                                      std::string &string_ret);

    E_EtherCANErrCode getFirmwareVersion(t_grid_state &gs,
                                         const t_fpuset &fpuset);

    E_EtherCANErrCode readSerialNumbers(t_grid_state &gs, const t_fpuset &fpuset);
    E_EtherCANErrCode writeSerialNumber(int fpu_id, const char *serial_number,
                                        t_grid_state &gs);

    E_EtherCANErrCode configMotion(const t_wtable &wavetable, t_grid_state &gs,
                        const t_fpuset &fpuset,
                        bool soft_protection = true,
                        bool allow_uninitialized = false,
                        int ruleset_version = DEFAULT_WAVEFORM_RULESET_VERSION,
                        bool warn_unsafe = true,
                        int verbosity = 3);

    E_EtherCANErrCode executeMotion(t_grid_state &gs, const t_fpuset &fpuset,
                                    bool sync_command = true);

    E_EtherCANErrCode abortMotion(t_grid_state &gs, const t_fpuset &fpuset,
                                  bool sync_command = true);

    E_EtherCANErrCode freeBetaCollision(int fpu_id,
                                        E_REQUEST_DIRECTION direction,
                                        t_grid_state &gs,
                                        bool soft_protection = true);
    E_EtherCANErrCode enableBetaCollisionProtection(t_grid_state &gs);

    E_EtherCANErrCode freeAlphaLimitBreach(int fpu_id,
                                           E_REQUEST_DIRECTION direction,
                                           t_grid_state &gs,
                                           bool soft_protection = true);
    E_EtherCANErrCode enableAlphaLimitProtection(t_grid_state &gs);

    E_EtherCANErrCode reverseMotion(t_grid_state &gs, const t_fpuset &fpuset,
                                    bool soft_protection = true);
    E_EtherCANErrCode repeatMotion(t_grid_state &gs, const t_fpuset &fpuset,
                                   bool soft_protection = true);

    void listAngles(const t_grid_state &gs, t_fpus_angles &fpus_angles_ret,
                    double alpha_datum_offset = ALPHA_DATUM_OFFSET,
                    bool show_uninitialized = false,
                    double asteps_per_deg = STEPS_PER_DEGREE_ALPHA,
                    double bsteps_per_deg = STEPS_PER_DEGREE_BETA);
    E_EtherCANErrCode countedAngles(t_grid_state &gs, const t_fpuset &fpuset,
                                    t_fpus_angles &fpus_angles_ret,
                                    bool show_uninitialized = false);

    E_EtherCANErrCode lockFPU(int fpu_id, t_grid_state &gs);
    E_EtherCANErrCode unlockFPU(int fpu_id, t_grid_state &gs);

    E_EtherCANErrCode enableMove(int fpu_id, t_grid_state &gs);

    E_EtherCANErrCode checkIntegrity(t_grid_state &gs, const t_fpuset &fpuset);

    // Static functions
    static void createFpuSetForSingleFpu(int fpu_id, t_fpuset &fpuset_ret);
#ifdef FLEXIBLE_CAN_MAPPING
    static void createFpuSetForIdList(const std::vector<int> &fpu_id_list,
                                      t_fpuset &fpuset_ret);
#else // NOT FLEXIBLE_CAN_MAPPING
    static void createFpuSetForNumFpus(int num_fpus, t_fpuset &fpuset_ret);
#endif // NOT FLEXIBLE_CAN_MAPPING

    //..........................................................................
protected:
    // NOTE: The following virtual functions are overridden in GridDriver

    virtual E_EtherCANErrCode _post_connect_hook()
    {
        return DE_OK;
    }

    // Reset hook functions
    virtual E_EtherCANErrCode _reset_hook(t_grid_state &old_state,
                                          t_grid_state &gs,
                                          const t_fpuset &fpuset)
    {
        UNUSED_ARG(old_state);
        UNUSED_ARG(gs);
        UNUSED_ARG(fpuset);

        return DE_OK;
    }
    virtual void _reset_counter_hook(double alpha_target, double beta_target,
                                     t_grid_state &old_state, t_grid_state &gs,
                                     const t_fpuset &fpuset)
    {
        UNUSED_ARG(alpha_target);
        UNUSED_ARG(beta_target);
        UNUSED_ARG(old_state);
        UNUSED_ARG(gs);
        UNUSED_ARG(fpuset);
    }

    // findDatum() hook functions
    // TODO: Do the t_grid_state's below need to be const? Or will they
    // possibly be altered inside the functions?
    virtual E_EtherCANErrCode _allow_find_datum_hook(t_grid_state &gs,
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
        return DE_OK;
    }
    virtual E_EtherCANErrCode _start_find_datum_hook(t_grid_state &gs,
                                        const t_datum_search_flags &search_modes,
                                        enum E_DATUM_SELECTION selected_arm,
                                        const t_fpuset &fpuset,
                                        t_fpu_positions &initial_positions,
                                        bool soft_protection)
    {
        UNUSED_ARG(gs);
        UNUSED_ARG(search_modes);
        UNUSED_ARG(selected_arm);
        UNUSED_ARG(fpuset);
        UNUSED_ARG(initial_positions);
        UNUSED_ARG(soft_protection);
        return DE_OK;
    }
    virtual E_EtherCANErrCode _cancel_find_datum_hook(t_grid_state &gs,
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
        return DE_OK;
    }
    virtual E_EtherCANErrCode _finished_find_datum_hook(
                                           const t_grid_state &prev_gs,
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
        return DE_OK;
    }

    // configMotion() hook functions
    virtual E_EtherCANErrCode _pre_config_motion_hook(const t_wtable &wtable,
                                                      t_grid_state &gs,
                                                      const t_fpuset &fpuset,
                                                      Range wmode = Range::Error)
    {
        UNUSED_ARG(wtable);
        UNUSED_ARG(gs);
        UNUSED_ARG(fpuset);
        UNUSED_ARG(wmode);
        return DE_OK;
    }
    virtual E_EtherCANErrCode _post_config_motion_hook(const t_wtable &wtable,
                                                       t_grid_state &gs,
                                                       const t_fpuset &fpuset)
    {
        // TODO: Add C++/Linux equivalent of Python version's "with self.lock" here 

        UNUSED_ARG(wtable);
        UNUSED_ARG(gs);
        set_wtable_reversed(fpuset, false);
        return DE_OK;
    }

    // repeatMotion() hook functions
    virtual E_EtherCANErrCode _pre_repeat_motion_hook(const t_wtable &wtable,
                                                      t_grid_state &gs,
                                                      const t_fpuset &fpuset,
                                                      Range wmode = Range::Error)
    {
        UNUSED_ARG(wtable);
        UNUSED_ARG(gs);
        UNUSED_ARG(fpuset);
        UNUSED_ARG(wmode);
        return DE_OK;
    }
    virtual E_EtherCANErrCode _post_repeat_motion_hook(const t_wtable &wtable,
                                                       t_grid_state &gs,
                                                       const t_fpuset &fpuset)
    {
        // TODO: Add C++/Linux equivalent of Python version's "with self.lock" here 

        UNUSED_ARG(wtable);
        UNUSED_ARG(gs);
        set_wtable_reversed(fpuset, false);
        return DE_OK;
    }

    // reverseMotion() hook functions
    virtual E_EtherCANErrCode _pre_reverse_motion_hook(const t_wtable &wtable,
                                                       t_grid_state &gs,
                                                       const t_fpuset &fpuset,
                                                       Range wmode = Range::Error)
    {
        UNUSED_ARG(wtable);
        UNUSED_ARG(gs);
        UNUSED_ARG(fpuset);
        UNUSED_ARG(wmode);
        return DE_OK;
    }  
    virtual E_EtherCANErrCode _post_reverse_motion_hook(const t_wtable &wtable,
                                                        t_grid_state &gs,
                                                        const t_fpuset &fpuset)
    {
        // TODO: Add C++/Linux equivalent of Python version's "with self.lock" here 

        UNUSED_ARG(wtable);
        UNUSED_ARG(gs);
        set_wtable_reversed(fpuset, true);
        return DE_OK;
    }

    // executeMotion() hook functions
    virtual E_EtherCANErrCode _start_execute_motion_hook(t_grid_state &gs,
                                                         const t_fpuset &fpuset,
                                            t_fpu_positions &initial_positions)
    {
        UNUSED_ARG(gs);
        UNUSED_ARG(fpuset);
        UNUSED_ARG(initial_positions);
        return DE_OK;
    }
    virtual E_EtherCANErrCode _cancel_execute_motion_hook(t_grid_state &gs,
                                                          const t_fpuset &fpuset,
                                    const t_fpu_positions &initial_positions)
    {
        UNUSED_ARG(gs);
        UNUSED_ARG(fpuset);
        UNUSED_ARG(initial_positions);
        return DE_OK;
    }
    virtual E_EtherCANErrCode _post_execute_motion_hook(t_grid_state &gs,
                                                const t_grid_state &old_gs,
                                                const t_grid_state &move_gs,
                                                const t_fpuset &fpuset)
    {
        UNUSED_ARG(gs);
        UNUSED_ARG(old_gs);
        UNUSED_ARG(move_gs);
        UNUSED_ARG(fpuset);
        return DE_OK;
    }

    // freeBetaCollision() hook functions
    virtual E_EtherCANErrCode _pre_free_beta_collision_hook(int fpu_id,
                                                E_REQUEST_DIRECTION direction,
                                                const t_grid_state &gs,
                                                bool soft_protection = true)
    {
        UNUSED_ARG(fpu_id);
        UNUSED_ARG(direction);
        UNUSED_ARG(gs);
        UNUSED_ARG(soft_protection);
        return DE_OK;
    }
    virtual E_EtherCANErrCode _post_free_beta_collision_hook(int fpu_id,
                                                E_REQUEST_DIRECTION direction,
                                                const t_grid_state &gs)
    {
        UNUSED_ARG(fpu_id);
        UNUSED_ARG(direction);
        UNUSED_ARG(gs);
        return DE_OK;
    }

    // freeAlphaLimitBreach() hook functions
    virtual E_EtherCANErrCode _pre_free_alpha_limit_breach_hook(int fpu_id,
                                                E_REQUEST_DIRECTION direction,
                                                const t_grid_state &gs,
                                                bool soft_protection = true)
    {
        UNUSED_ARG(fpu_id);
        UNUSED_ARG(direction);
        UNUSED_ARG(gs);
        UNUSED_ARG(soft_protection);
        return DE_OK;
    }
    virtual E_EtherCANErrCode _post_free_alpha_limit_breach_hook(int fpu_id,
                                                E_REQUEST_DIRECTION direction,
                                                const t_grid_state &gs)
    {
        UNUSED_ARG(fpu_id);
        UNUSED_ARG(direction);
        UNUSED_ARG(gs);
        return DE_OK;
    }

    // Error counter functions
    void updateErrorCountersForFpuSet(const t_grid_state &prev_gs,
                                      const t_grid_state &gs,
                                      const t_fpuset &fpuset,
                                      bool datum_cmd = false);
    virtual void _update_error_counters(FpuCounters &fpu_counters,
                                        const t_fpu_state &prev_fpu_state,
                                        const t_fpu_state &moved_fpu_state,
                                        bool datum_cmd = false)
    {
        UNUSED_ARG(fpu_counters);
        UNUSED_ARG(prev_fpu_state);
        UNUSED_ARG(moved_fpu_state);
        UNUSED_ARG(datum_cmd);
    }

    E_EtherCANErrCode checkInitializedAndFpuset(const t_fpuset &fpuset);
    void need_ping(const t_grid_state &gs, const t_fpuset &fpuset,
                   t_fpuset &pingset_ret);
    E_EtherCANErrCode _pingFPUs(t_grid_state &gs, const t_fpuset &fpuset);
    // N.B. static function
    static bool wavetable_was_received(const t_wtable &wtable, int fpu_id,
                                       const t_fpu_state &fpu_state,  
                                       bool allow_unconfirmed = false,
                                E_FPU_STATE target_state = FPST_READY_FORWARD);
    void buildWtableFromLastWaveforms(const t_fpuset &fpuset,
                                      t_wtable &wtable_ret);
    void sleepSecs(double seconds);

    E_EtherCANErrCode check_fpuset(const t_fpuset &fpuset);

    // Config: Only the constructor and initialize() must write this - do NOT
    // write at all after initialize() has been called
    EtherCANInterfaceConfig config;
    bool initialize_was_called_ok = false;

    // NOTE: wf_reversed is not used in UnprotectedGridDriver, so it has been
    // moved to GridDriver (N.B. The original Python version had it in
    // UnprotectedGridDriver)
    // std::vector<bool> wf_reversed;

    // _gd: N.B. Can't use a unique_ptr for EtherCANInterface, probably
    // because it's explicitly non-movable
    // TODO: Rename to something like etherCanIfPtr eventually - it's named as
    // _gd so that it's the same as in FpuGridDriver.py for now
    EtherCANInterface *_gd = nullptr;

    // FpuData: Per-FPU data. NOTE: Mostly used only by the GridDriver class
    // which inherits from this UnprotectedGridDriver class - only the
    // fpus_data[n].db.last_waveform and pus_data[n].db.counters elements are
    // used here.
    // TODO: The entire structure is included here only so that
    // UnprotectedGridDriver can access fpus_data[n].db.last_waveform - this
    // is not ideal, but might be rationalised in future if
    // UnprotectedGridDriver and GridDriver are merged into one class
    // eventually?
    struct FpuData
    {
        FpuDbData db;

        Interval a_caloffset;
        Interval b_caloffset;
        FpuCounters _last_counters;
        t_fpu_position target_position;
    };

    std::vector<FpuData> fpus_data;

    //..........................................................................
private:
    E_EtherCANErrCode pingIfNeeded(t_grid_state &gs, const t_fpuset &fpuset);

    void set_wtable_reversed(const t_fpuset &fpuset, bool is_reversed = false);

    bool wavetables_incomplete = false;

    // TODO: Add locked_gateways here, but need to convert Python devicelock.py
    // to C++ first (and use a Linux named semaphore instead of a lock file for
    // this?)

    //..........................................................................
};


//==============================================================================

} // namespace mpifps

#endif // UNPROTECTEDGRIDDRIVER_H
