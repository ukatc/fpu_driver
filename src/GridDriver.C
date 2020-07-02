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
// NAME GridDriver.C
//
// TODO: Put description here
//
////////////////////////////////////////////////////////////////////////////////


// ********** NOTE: This file (along with UnprotectedGridDriver.C) is Bart's
// work in progress for converting the classes and functions in FPUGridDriver.py
// from Python to C++.

#include "GridDriver.h"

#ifdef DEBUG
#include <stdio.h>
#endif

namespace mpifps
{


//==============================================================================

void GridDriver::_post_connect_hook(const EtherCANInterfaceConfig &config)
{
    // TODO
}

//------------------------------------------------------------------------------

void GridDriver::_allow_find_datum_hook(t_grid_state &gs,
                                        t_datum_search_flags &search_modes,
                                        enum E_DATUM_SELECTION selected_arm,
                                        const t_fpuset &fpuset,
                                        bool support_uninitialized_auto)
{
    // TODO
}

void GridDriver::_start_find_datum_hook(t_grid_state &gs,
                                        const t_datum_search_flags &search_modes,
                                        enum E_DATUM_SELECTION selected_arm,
                                        const t_fpuset &fpuset,
                                        FpuPositions &initial_positions_ret,
                                        bool soft_protection)
{
    // TODO
}

void GridDriver::_cancel_find_datum_hook(t_grid_state &gs, 
                                         const t_fpuset &fpuset,
                                         const FpuPositions &initial_positions)
{
    // TODO
}

void GridDriver::_finished_find_datum_hook(t_grid_state &prev_gs,
                                           t_grid_state &datum_gs,
                                           const t_datum_search_flags &search_modes,
                                           const t_fpuset &fpuset,
                                           bool was_cancelled,
                                           const FpuPositions &initial_positions, 
                                           enum E_DATUM_SELECTION selected_arm)
{
    // TODO
}

//------------------------------------------------------------------------------

void GridDriver::_update_error_counters(const t_fpu_state &prev_fpu,
                                        const t_fpu_state &moved_fpu,
                                        bool datum_cmd)
{

    // TODO
    // N.B. Also see the Python GridDriver::_update_error_counters() function,
    // and my new FpuErrorCounterType enum class in GridDriver.h for this

}

//------------------------------------------------------------------------------

void GridDriver::_pre_config_motion_hook(const t_wtable &wtable,
                                         t_grid_state &gs,
                                         const t_fpuset &fpuset, Range wmode)
{
    // TODO
}

void GridDriver::_post_config_motion_hook(const t_wtable &wtable, 
                                          t_grid_state &gs,
                                          const t_fpuset &fpuset)
{
    // TODO
}

//------------------------------------------------------------------------------

void GridDriver::_start_execute_motion_hook(t_grid_state &gs,
                                            const t_fpuset &fpuset,
                                            const FpuPositions &initial_positions)
{
    // TODO
}

void GridDriver::_cancel_execute_motion_hook(t_grid_state &gs,
                                             const t_fpuset &fpuset,
                                             const FpuPositions &initial_positions)
{
    // TODO
}

void GridDriver::_post_execute_motion_hook(t_grid_state &gs,
                                           const t_grid_state &old_gs,
                                           const t_grid_state &move_gs,
                                           const t_fpuset &fpuset)
{
    // TODO
}

//..............................................................................
// TODO: Boost.Python wrapper test member functions - remove eventually
//..............................................................................

int GridDriver::boostPythonIncrement()
{
    dummyCounter++;
    return dummyCounter;
}

double GridDriver::boostPythonDivide(double dividend, double divisor)
{
    return dividend / divisor;
}

int GridDriver::boostPythonGetNumFPUs()
{
    return config.num_fpus;
}


//==============================================================================

} // namespace mpifps

