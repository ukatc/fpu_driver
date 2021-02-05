// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-06-15  Created (translated from Python FpuGridDriver.py).
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME GridDriver.h
//
// This GridDriver class adds a software protection layer on top of the basic
// UnprotectedGridDriver class.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef GRIDDRIVER_H
#define GRIDDRIVER_H

//******************************
// TODO: For testing only - enable to try an FPU (with first DIP switch set
// and the others off) on the 2nd CAN bus on the EtherCAN box (labelled as "1",
// where the first one is labelled as "0"), and with the FPU already set up in
// the FPU database.
// N.B. This FPU would normally work when its fpu ID is 0 on CAN bus 0, but
// when connect backplane to EtherCAN box's CAN bus 1 then its fpu ID will be
// 76 (because 2 backplanes on each CAN bus, with 38 FPUs on each backplane)
//#define USE_2ND_CANBUS
#ifdef USE_2ND_CANBUS
#define NEXT_CANBUS_FPU_ID (76)
#endif
//******************************



// TODO: ENABLE_PROTECTION_CODE macro - remove once done with it
// Define this macro in a project's global predefined symbols to enable the
// protection code work-in-progress, or disable it so that can continue to use
// the unprotected code for the time being. If disabled then also allows the
// current build_griddriver_wrapped.sh script to build the Boost.Python-wrapped
// driver successfully (otherwise, will fail because the script doesn't yet
// add in the required LMDB files and so forth).

#include <map>
#include <vector>
#include <string>
#include "UnprotectedGridDriver.h"
#ifdef ENABLE_PROTECTION_CODE
#include "ProtectionDB.h"
#endif // ENABLE_PROTECTION_CODE
#include "Interval.h"
#include "FPUCounters.h"


namespace mpifps
{

// Forward reference for friend-ing in UnprotectedGridDriver below
class GridDriverTester;

//==============================================================================

class GridDriver : public UnprotectedGridDriver
{
    //..........................................................................
public:
    // Declare test class as friend so that it can access protected/private
    // member variables and functions
    friend class GridDriverTester;
  
    using UnprotectedGridDriver::UnprotectedGridDriver; // Inherit constructor

    ~GridDriver();

    E_EtherCANErrCode initProtection(bool use_mockup_db = false);

    bool initializedOk() override;

#ifdef ENABLE_PROTECTION_CODE
    E_EtherCANErrCode pingFPUs(t_grid_state &gs, const t_fpuset &fpuset) override;
    E_EtherCANErrCode trackedAnglesVals(const t_grid_state &gs,
                                        const t_fpuset &fpuset,
                                        t_fpu_positions &positions_ret);
    E_EtherCANErrCode trackedAnglesString(const t_grid_state &gs,
                                          const t_fpuset &fpuset,
                                          std::string &return_string,
                                          bool show_offsets = false,
                                          bool active = false);
#endif // ENABLE_PROTECTION_CODE

    // TODO: Is a destructor needed?

    //..........................................................................
private:
    bool initprotection_was_called_ok = false;

#ifdef ENABLE_PROTECTION_CODE

    //.............................
    // TODO: Are these data structures the correct equivalents of the original
    // Python code? Also, see comments above the t_fpu_positions definition in
    // UnprotectedGridDriver.h

    // TODO: Python version's GridDriver::__init__() has "with self.lock"
    // around its XXXX_ranges initialisations - is this superfluous because
    // these are created in the constructor and therefore won't have any
    // chance of being accessed simultaneously from another instance - OR,
    // are these data structures shared between multiple instances somehow?

    // Position intervals which are being configured by configMotion
    t_fpu_positions configuring_ranges;
    // Position intervals which have successfully been configured and will
    // become valid with next executeMotion
    t_fpu_positions configured_ranges;

    t_fpu_positions configuring_targets;
    t_fpu_positions configured_targets;

    //.............................
    
    ProtectionDB protection_db;

    double _alpha_angle(const t_fpu_state &fpu_state, bool &alpha_underflow_ret,
                        bool &alpha_overflow_ret);
    double _beta_angle(const t_fpu_state &fpu_state, bool &beta_underflow_ret,
                       bool &beta_overflow_ret);

    // TODO: Not needed? See comments above disabled set_wtable_reversed() in
    // GridDriver.C
    // void set_wtable_reversed(const t_fpuset &fpuset, bool is_reversed = false);

    // The following hook functions override those in UnprotectedGridDriver

    E_EtherCANErrCode _post_connect_hook() override;

    E_EtherCANErrCode _reset_hook(t_grid_state &old_state, t_grid_state &gs,
                                  const t_fpuset &fpuset) override;
    void _reset_counter_hook(double alpha_target, double beta_target,
                             t_grid_state &old_state, t_grid_state &gs,
                             const t_fpuset &fpuset) override;

    bool _update_apos(const std::unique_ptr<ProtectionDbTxn> &txn,
                      const char *serial_number, int fpu_id,
                      const Interval &new_apos, bool store = true);
    bool _update_bpos(const std::unique_ptr<ProtectionDbTxn> &txn,
                      const char *serial_number, int fpu_id,
                      const Interval &new_bpos, bool store = true);

    E_EtherCANErrCode _refresh_positions(const t_grid_state &grid_state,
                                         bool store, const t_fpuset &fpuset);

    // findDatum() hook functions
    E_EtherCANErrCode _allow_find_datum_hook(t_grid_state &gs,
                                    t_datum_search_flags &search_modes,
                                    enum E_DATUM_SELECTION selected_arm,
                                    const t_fpuset &fpuset,
                                    bool support_uninitialized_auto) override;
    E_EtherCANErrCode _start_find_datum_hook(t_grid_state &gs,
                                    const t_datum_search_flags &search_modes,
                                    enum E_DATUM_SELECTION selected_arm,
                                    const t_fpuset &fpuset,
                                    t_fpu_positions &initial_positions,
                                    bool soft_protection) override;
    E_EtherCANErrCode _cancel_find_datum_hook(t_grid_state &gs,
                            const t_fpuset &fpuset,
                            const t_fpu_positions &initial_positions) override;
    E_EtherCANErrCode _finished_find_datum_hook(const t_grid_state &prev_gs,
                            t_grid_state &datum_gs,
                            const t_datum_search_flags &search_modes,
                            const t_fpuset &fpuset, bool was_cancelled,
                            const t_fpu_positions &initial_positions, // TODO: Not used (in Python version) - remove?
                            enum E_DATUM_SELECTION selected_arm) override;

    void _update_counters_find_datum(FpuCounters &fpu_counters,
                                     const t_fpu_state &prev_fpu_state,
                                     const t_fpu_state &datum_fpu_state);

    E_EtherCANErrCode _check_allowed_range(int fpu_id, int stepnum,
                                           const char *arm_name,
                                           const Interval &xlimits,
                                           const Interval &xpos,
                                           Interval &new_range, Range wmode);

    E_EtherCANErrCode _check_and_register_wtable(const t_wtable &wtable,
                                                 t_grid_state &gs,
                                                 const t_fpuset &fpuset,
                                                 Range wmode, int sign);

    void _update_error_counters(FpuCounters &fpu_counters,
                                const t_fpu_state &prev_fpu_state,
                                const t_fpu_state &moved_fpu_state,
                                bool datum_cmd = false) override;

    // configMotion() hook functions
    E_EtherCANErrCode _pre_config_motion_hook(const t_wtable &wtable,
                                              t_grid_state &gs,
                                              const t_fpuset &fpuset,
                                              Range wmode) override;
    E_EtherCANErrCode _save_wtable_direction(const t_fpuset &fpuset,
                                             bool is_reversed,
                                             t_grid_state &gs);
    E_EtherCANErrCode _post_config_motion_hook(const t_wtable &wtable,
                                               t_grid_state &gs,
                                               const t_fpuset &fpuset) override;

    // repeatMotion() and reverseMotion() hook functions
    E_EtherCANErrCode _pre_repeat_motion_hook(const t_wtable &wtable,
                                              t_grid_state &gs,
                                              const t_fpuset &fpuset,
                                              Range wmode = Range::Error) override;
    E_EtherCANErrCode _post_repeat_motion_hook(const t_wtable &wtable,
                                               t_grid_state &gs,
                                               const t_fpuset &fpuset) override;
    E_EtherCANErrCode _pre_reverse_motion_hook(const t_wtable &wtable,
                                               t_grid_state &gs,
                                               const t_fpuset &fpuset,
                                               Range wmode = Range::Error) override;
    E_EtherCANErrCode _post_reverse_motion_hook(const t_wtable &wtable,
                                                t_grid_state &gs,
                                                const t_fpuset &fpuset) override;
    E_EtherCANErrCode _post_repeat_reverse_motion_hook(const t_wtable &wtable,
                                                       t_grid_state &gs,
                                                       const t_fpuset &fpuset,
                                                       E_FPU_STATE target_state,
                                                       bool is_reversed);

    // executeMotion() hook functions
    E_EtherCANErrCode _start_execute_motion_hook(t_grid_state &gs,
                                                 const t_fpuset &fpuset,
                                t_fpu_positions &initial_positions) override;
    E_EtherCANErrCode _cancel_execute_motion_hook(t_grid_state &gs,
                                                  const t_fpuset &fpuset,
                            const t_fpu_positions &initial_positions) override;
    E_EtherCANErrCode _post_execute_motion_hook(t_grid_state &gs,
                                                const t_grid_state &old_gs,
                                                const t_grid_state &move_gs,
                                                const t_fpuset &fpuset) override;

    void _update_counters_execute_motion(int fpu_id, FpuCounters &fpu_counters,
                                         const t_waveform_steps &waveform,
                                         bool is_reversed,
                                         bool cancel = false);

    // freeBetaCollision() hook functions
    E_EtherCANErrCode _pre_free_beta_collision_hook(int fpu_id,
                                        E_REQUEST_DIRECTION direction,
                                        const t_grid_state &gs,
                                        bool soft_protection = true) override;
    E_EtherCANErrCode _post_free_beta_collision_hook(int fpu_id,
                                        E_REQUEST_DIRECTION direction,
                                        const t_grid_state &gs) override;

    // freeAlphaLimitBreach() hook functions
    E_EtherCANErrCode _pre_free_alpha_limit_breach_hook(int fpu_id,
                                        E_REQUEST_DIRECTION direction,
                                        const t_grid_state &gs,
                                        bool soft_protection = true) override;
    E_EtherCANErrCode _post_free_alpha_limit_breach_hook(int fpu_id,
                                        E_REQUEST_DIRECTION direction,
                                        const t_grid_state &gs) override;

    void getDuplicateSerialNumbers(t_grid_state &grid_state,
                        std::vector<std::string> &duplicate_snumbers_ret);

    int sign(int64_t val);

#endif // ENABLE_PROTECTION_CODE

    //..........................................................................

};

//==============================================================================

} // namespace mpifps

#endif // GRIDDRIVER_H
