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

// TODO: ENABLE_PROTECTION_CODE macro
// Define this macro in a project's global predefined symbols to enable the
// protection code work-in-progress, or disable it so that can continue to use
// the unprotected code for the time being.
// TODO: Remove this macro once done

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

    E_EtherCANErrCode initProtection(bool mockup = false);

    bool initializedOk() override;

    ProtectionDB &getProtectionDB();
    
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

#ifdef FPU_DB_DATA_AGGREGATED
    struct FpuData
    {
        FpuDbData db;

        Interval a_caloffset;
        Interval b_caloffset;
        FpuCounters _last_counters;
        t_fpu_position target_position;
    };

    std::vector<FpuData> fpus_data;

#else // NOT FPU_DB_DATA_AGGREGATED
    // N.B. These vectors all have their sizes set to the number of FPUs
    std::vector<Interval> apositions;
    std::vector<Interval> bpositions;
    // TODO: This wf_reversed vector was moved here into GridDriver from
    // UnprotectedGridDriver so that it can eventually be included into an FPU
    // database data structure - this is OK because it's not actually used in
    // UnprotectedGridDriver. N.B. The associated set_wtable_reversed() function
    // is no longer required so has been removed, but the getReversed() function
    // might still be required? (it's shown in the FPU grid driver document)
    std::vector<bool> wf_reversed; // N.B. Size is set to config.num_fpus
    std::vector<Interval> alimits;
    std::vector<Interval> blimits;
    std::vector<int64_t> maxaretries;
    std::vector<int64_t> aretries_cw;
    std::vector<int64_t> aretries_acw;
    std::vector<int64_t> maxbretries;
    std::vector<int64_t> bretries_cw;
    std::vector<int64_t> bretries_acw;
    std::vector<FpuCounters> counters;

    std::vector<Interval> a_caloffsets;
    std::vector<Interval> b_caloffsets;
    std::vector<FpuCounters> _last_counters;
    std::vector<t_fpu_position> target_positions;
#endif // NOT FPU_DB_DATA_AGGREGATED
#endif // ENABLE_PROTECTION_CODE

    // Variable-sized maps? (for now)
    // TODO: See comments above the t_fpu_positions definition in
    // UnprotectedGridDriver.h
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

    // TODO: Not needed: See comments above disabled set_wtable_reversed() in
    // GridDriver.C
    // void set_wtable_reversed(const t_fpuset &fpuset, bool is_reversed = false);

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
                                         const t_waveform_steps &waveform,
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
