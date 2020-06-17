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

GridDriver::GridDriver()
{
    
}

void GridDriver::_post_connect_hook(const EtherCANInterfaceConfig &config)
{
    // TODO
}

void GridDriver::_allow_find_datum_hook(t_grid_state &gs,
                    AsyncInterface::t_datum_search_flags &search_modes,
                    enum E_DATUM_SELECTION selected_arm,
                    const AsyncInterface::t_fpuset &fpuset,
                    bool support_uninitialized_auto)
{
    // TODO
}

void GridDriver::_start_find_datum_hook(t_grid_state &gs,
                    const AsyncInterface::t_datum_search_flags &search_modes,
                    enum E_DATUM_SELECTION selected_arm,
                    const AsyncInterface::t_fpuset &fpuset,
                    FpuPositions &initial_positions_ret, bool soft_protection)
{
    // TODO
}

void GridDriver::_cancel_find_datum_hook(t_grid_state &gs,
                    const AsyncInterface::t_fpuset &fpuset,
                    const FpuPositions &initial_positions)
{
    // TODO
}

void GridDriver::_finished_find_datum_hook(t_grid_state &prev_gs,
                    t_grid_state &datum_gs,
                    const AsyncInterface::t_datum_search_flags &search_modes,
                    const AsyncInterface::t_fpuset &fpuset,
                    bool was_cancelled, const FpuPositions &initial_positions, 
                    enum E_DATUM_SELECTION selected_arm)
{
    // TODO
}


//==============================================================================


} // namespace mpifps

