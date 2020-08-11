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
    // TODO: The following data structures are my initial WIP best guesses -
    // look at these further

    struct FpuArmPos
    {
        FpuArmPos()
        {
            datum_offset = 0.0;
        }

        Interval position;
        double datum_offset;
    };

    FpuArmPos apositions[MAX_NUM_POSITIONERS];
    FpuArmPos bpositions[MAX_NUM_POSITIONERS];

    // last_wavetable: TODO: ALREADY IN UnprotectedGridDriver - see notes in my doc
    // wf_reversed: TODO: ALREADY IN UnprotectedGridDriver - see notes in my doc

    FpuArmPos alimits[MAX_NUM_POSITIONERS];
    FpuArmPos blimits[MAX_NUM_POSITIONERS];

    Interval a_caloffsets[MAX_NUM_POSITIONERS];
    Interval b_caloffsets[MAX_NUM_POSITIONERS];

    int64_t aretries_cw[MAX_NUM_POSITIONERS];
    int64_t aretries_acw[MAX_NUM_POSITIONERS];
    int64_t bretries_cw[MAX_NUM_POSITIONERS];
    int64_t bretries_acw[MAX_NUM_POSITIONERS];

    FpuCounters counters[MAX_NUM_POSITIONERS];
    FpuCounters _last_counters[MAX_NUM_POSITIONERS];

    struct
    {
        FpuArmPos apos;
        FpuArmPos bpos;
    } target_positions[MAX_NUM_POSITIONERS];

    // Variable-sized maps
    // TODO: Is it correct that these will be variable-sized? Do any of the
    // arrays above need to be variable-sized maps as well, rather than
    // fixed-size arrays?
    // TODO: Figure out configuring_targets and configured_targets - see my doc
#if 0
    // TODO: Need data structure like the following (I think)
    std::map<int, {FpuArmPos, FpuArmPos}> configuring_targets; // <fpu_id, 
#endif // 0

    //*****************************
    //*****************************

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

    //..........................................................................

};

//==============================================================================

} // namespace mpifps

#endif // GRIDDRIVER_H
