// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-06-15  TODO: Put comment here
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME GridDriver.h
//
// TODO: Put description here
//
////////////////////////////////////////////////////////////////////////////////

#ifndef GRIDDRIVER_H
#define GRIDDRIVER_H

#include <map>
#include <vector>
#include <string>
#include "UnprotectedGridDriver.h"
#ifdef ENABLE_PROTECTION_CODE
#include "ProtectionDB.h"
#endif // ENABLE_PROTECTION_CODE
#include "Interval.h"

// ENABLE_PROTECTION_CODE macro note: Define it (in a project's global
// predefined symbols) to enable the protection code work-in-progress, or
// disable it so that can continue to use the unprotected code for the time
// being.


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

    E_EtherCANErrCode initProtection(bool mockup = false);

    bool initializedOk() override;

    E_EtherCANErrCode pingFPUs(t_grid_state &gs, const t_fpuset &fpuset) override;

    //............................................
    // TODO: Test function for Boost.Python wrapper experimentation only -
    // remove when no longer needed
    double boostPythonDivide(double dividend, double divisor);
    //............................................

    // TODO: Is a destructor needed?

    //..........................................................................
private:
    // Error counters functionality
    enum class FpuErrorCounterType
    {
        Collisions = 0,
        LimitBreaches,
        CanTimeouts,
        DatumTimeouts,
        MovementTimeouts,

        NumFpuErrorCounterTypes
    };

    bool initprotection_was_called_ok = false;

#ifdef ENABLE_PROTECTION_CODE

    //*****************************
    //*****************************
    // TODO: The following data structures are my initial WIP best guesses
    // converted from their Python equivalents as shown in
    // _post_connect_hook() - check these further

    Interval apositions[MAX_NUM_POSITIONERS];
    Interval bpositions[MAX_NUM_POSITIONERS];

    // NOTE: last_wavetable and wf_reversed are inherited from
    // UnprotectedGridDriver, so don't need to define them here
    // TODO: Remove the comment above once fully converted from
    // Python

    Interval alimits[MAX_NUM_POSITIONERS];
    Interval blimits[MAX_NUM_POSITIONERS];

    Interval a_caloffsets[MAX_NUM_POSITIONERS];
    Interval b_caloffsets[MAX_NUM_POSITIONERS];

    int64_t aretries_cw[MAX_NUM_POSITIONERS];
    int64_t aretries_acw[MAX_NUM_POSITIONERS];
    int64_t bretries_cw[MAX_NUM_POSITIONERS];
    int64_t bretries_acw[MAX_NUM_POSITIONERS];

    FpuCounters counters[MAX_NUM_POSITIONERS];
    FpuCounters _last_counters[MAX_NUM_POSITIONERS];

    // Variable-sized maps (for now)
    // TODO: See comments above the t_fpu_positions definition in
    // UnprotectedGridDriver.h
    t_fpu_positions target_positions;

#endif // ENABLE_PROTECTION_CODE
    
    t_fpu_positions configuring_targets;
    t_fpu_positions configured_targets;

    //*****************************
    //*****************************
    
#ifdef ENABLE_PROTECTION_CODE

    ProtectionDB protection_db;

    double _alpha_angle(const t_fpu_state &fpu_state, bool &alpha_underflow_ret,
                        bool &alpha_overflow_ret);
    double _beta_angle(const t_fpu_state &fpu_state, bool &beta_underflow_ret,
                       bool &beta_overflow_ret);
#endif // ENABLE_PROTECTION_CODE

    // The following hook functions override those in UnprotectedGridDriver

    void _post_connect_hook() override;

    // findDatum() hook functions
    void _allow_find_datum_hook(t_grid_state &gs,
                                t_datum_search_flags &search_modes,
                                enum E_DATUM_SELECTION selected_arm,
                                const t_fpuset &fpuset,
                                bool support_uninitialized_auto) override;
    void _start_find_datum_hook(t_grid_state &gs,
                                const t_datum_search_flags &search_modes,
                                enum E_DATUM_SELECTION selected_arm,
                                const t_fpuset &fpuset,
                                t_fpu_positions &initial_positions_ret,
                                bool soft_protection) override;
    void _cancel_find_datum_hook(t_grid_state &gs, const t_fpuset &fpuset,
                                 const t_fpu_positions &initial_positions) override;
    void _finished_find_datum_hook(t_grid_state &prev_gs, t_grid_state &datum_gs,
                                   const t_datum_search_flags &search_modes,
                                   const t_fpuset &fpuset, bool was_cancelled,
                                   const t_fpu_positions &initial_positions, // TODO: Not used (in Python version) - remove?
                                   enum E_DATUM_SELECTION selected_arm) override;

    // resetFPUs() hook function
    void _reset_hook(t_grid_state &old_state, t_grid_state &gs,
                     const t_fpuset &fpuset) override;

    E_EtherCANErrCode _refresh_positions(t_grid_state &grid_state, bool store,
                                         const t_fpuset &fpuset);

    void _update_error_counters(const t_fpu_state &prev_fpu,
                                const t_fpu_state &moved_fpu,
                                bool datum_cmd = false) override;

    // configMotion() hook functions
    void _pre_config_motion_hook(const t_wtable &wtable, t_grid_state &gs,
                                 const t_fpuset &fpuset, Range wmode) override;
    void _post_config_motion_hook(const t_wtable &wtable, t_grid_state &gs,
                                  const t_fpuset &fpuset) override;

    // executeMotion() hook functions
    void _start_execute_motion_hook(t_grid_state &gs, const t_fpuset &fpuset,
                                    const t_fpu_positions &initial_positions) override;
    void _cancel_execute_motion_hook(t_grid_state &gs, const t_fpuset &fpuset,
                                     const t_fpu_positions &initial_positions) override;
    void _post_execute_motion_hook(t_grid_state &gs, const t_grid_state &old_gs,
                                   const t_grid_state &move_gs,
                                   const t_fpuset &fpuset) override;
#ifdef ENABLE_PROTECTION_CODE
    void _update_counters_execute_motion(int fpu_id, FpuCounters &fpu_counters,
                                         const t_waveform &waveform,
                                         bool is_reversed,
                                         bool cancel = false);
#endif // ENABLE_PROTECTION_CODE

    void getFpuSetForConfigNumFpus(t_fpuset &fpuset_ret);

    void getDuplicateSerialNumbers(t_grid_state &grid_state,
                        std::vector<std::string> &duplicate_snumbers_ret);

    int sign(int64_t val);

    //..........................................................................

};

//==============================================================================

} // namespace mpifps

#endif // GRIDDRIVER_H
