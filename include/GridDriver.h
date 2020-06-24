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

#include "UnprotectedGridDriver.h"

namespace mpifps
{


//==============================================================================

class GridDriver : public UnprotectedGridDriver
{
    //..........................................................................
public:
    // TODO: dummy_val is just a placeholder for now - need to implement
    // properly
    GridDriver(int dummy_val);

    //............................................
    // TODO: Ad-hoc functions for Boost.Python wrapper testing only - remove
    // when no longer needed
    int boostPythonIncrement();
    double boostPythonDivide(double dividend, double divisor);
    int boostPythonGetNumFPUs();
    // TODO: Ad-hoc test variable only - remove when no longer needed
    int dummyCounter = 0;
    //............................................

    // TODO: Is a destructor needed?

    //..........................................................................
private:    
    // The following hook functions override those in UnprotectedGridDriver

    void _post_connect_hook(const EtherCANInterfaceConfig &config) override;

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
                                FpuPositions &initial_positions_ret,
                                bool soft_protection) override;
    void _cancel_find_datum_hook(t_grid_state &gs, const t_fpuset &fpuset,
                                 const FpuPositions &initial_positions) override;
    void _finished_find_datum_hook(t_grid_state &prev_gs, t_grid_state &datum_gs,
                                   const t_datum_search_flags &search_modes,
                                   const t_fpuset &fpuset, bool was_cancelled,
                                   const FpuPositions &initial_positions, // TODO: Not used (in Python version) - remove?
                                   enum E_DATUM_SELECTION selected_arm) override;

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
                                    const FpuPositions &initial_positions) override;
    void _cancel_execute_motion_hook(t_grid_state &gs, const t_fpuset &fpuset,
                                     const FpuPositions &initial_positions) override;
    void _post_execute_motion_hook(t_grid_state &gs, const t_grid_state &old_gs,
                                   const t_grid_state &move_gs,
                                   const t_fpuset &fpuset) override;

    //..........................................................................

};

//==============================================================================

} // namespace mpifps

#endif // GRIDDRIVER_H
