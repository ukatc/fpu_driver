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

// ENABLE_PROTECTION_CODE macro: Define it (in a project's global predefined
// symbols) to enable the protection code work-in-progress, or disable it so
// that can continue to use the unprotected code for the time being.


#include "GridDriver.h"

#ifdef DEBUG
#include <stdio.h>
#endif

namespace mpifps
{


//==============================================================================
E_EtherCANErrCode GridDriver::initDb(bool mockup)
{
    // TODO: Temporary only
    UNUSED_ARG(mockup);

#ifdef ENABLE_PROTECTION_CODE
    if (initdb_was_called_ok)
    {
        return DE_INTERFACE_ALREADY_INITIALIZED;
    }

    if (!initialize_was_called_ok)
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }


    // TODO - implement the rest of this function

#endif // ENABLE_PROTECTION_CODE

    initdb_was_called_ok = true;
    return DE_OK;
}

//------------------------------------------------------------------------------
bool GridDriver::initializedOk()
{
    if ((initialize_was_called_ok) && (initdb_was_called_ok))
    {
        return true;
    }
    return false;
}

//------------------------------------------------------------------------------
void GridDriver::_post_connect_hook()
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

    // Temporary for now
    UNUSED_ARG(gs);
    UNUSED_ARG(search_modes);
    UNUSED_ARG(selected_arm);
    UNUSED_ARG(fpuset);
    UNUSED_ARG(support_uninitialized_auto);
}

//------------------------------------------------------------------------------
void GridDriver::_start_find_datum_hook(t_grid_state &gs,
                                        const t_datum_search_flags &search_modes,
                                        enum E_DATUM_SELECTION selected_arm,
                                        const t_fpuset &fpuset,
                                        t_fpu_positions &initial_positions_ret,
                                        bool soft_protection)
{
    // TODO

    // Temporary for now
    UNUSED_ARG(gs);
    UNUSED_ARG(search_modes);
    UNUSED_ARG(selected_arm);
    UNUSED_ARG(fpuset);
    UNUSED_ARG(initial_positions_ret);
    UNUSED_ARG(soft_protection);
}

//------------------------------------------------------------------------------
void GridDriver::_cancel_find_datum_hook(t_grid_state &gs, 
                                         const t_fpuset &fpuset,
                                         const t_fpu_positions &initial_positions)
{
    // TODO

    // Temporary for now
    UNUSED_ARG(gs);
    UNUSED_ARG(fpuset);
    UNUSED_ARG(initial_positions);
}

//------------------------------------------------------------------------------
void GridDriver::_finished_find_datum_hook(t_grid_state &prev_gs,
                                           t_grid_state &datum_gs,
                                           const t_datum_search_flags &search_modes,
                                           const t_fpuset &fpuset,
                                           bool was_cancelled,
                                           const t_fpu_positions &initial_positions, 
                                           enum E_DATUM_SELECTION selected_arm)
{
    // TODO

    // Temporary for now
    UNUSED_ARG(prev_gs);
    UNUSED_ARG(datum_gs);
    UNUSED_ARG(search_modes);
    UNUSED_ARG(fpuset);
    UNUSED_ARG(was_cancelled);
    UNUSED_ARG(initial_positions);
    UNUSED_ARG(selected_arm);
}

//------------------------------------------------------------------------------
void GridDriver::_reset_hook(t_grid_state &old_state, t_grid_state &gs,
                             const t_fpuset &fpuset)
{
    // TODO

    // Temporary for now
    UNUSED_ARG(old_state);
    UNUSED_ARG(gs);
    UNUSED_ARG(fpuset);
}

//------------------------------------------------------------------------------
void GridDriver::_update_error_counters(const t_fpu_state &prev_fpu,
                                        const t_fpu_state &moved_fpu,
                                        bool datum_cmd)
{

    // TODO
    // N.B. Also see the Python GridDriver::_update_error_counters() function,
    // and my new FpuErrorCounterType enum class in GridDriver.h for this

    // Temporary for now
    UNUSED_ARG(prev_fpu);
    UNUSED_ARG(moved_fpu);
    UNUSED_ARG(datum_cmd);
}

//------------------------------------------------------------------------------
void GridDriver::_pre_config_motion_hook(const t_wtable &wtable,
                                         t_grid_state &gs,
                                         const t_fpuset &fpuset, Range wmode)
{
    // TODO

    // Temporary for now
    UNUSED_ARG(wtable);
    UNUSED_ARG(gs);
    UNUSED_ARG(fpuset);
    UNUSED_ARG(wmode);
}

//------------------------------------------------------------------------------
void GridDriver::_post_config_motion_hook(const t_wtable &wtable, 
                                          t_grid_state &gs,
                                          const t_fpuset &fpuset)
{
    // TODO

    // Temporary for now
    UNUSED_ARG(wtable);
    UNUSED_ARG(gs);
    UNUSED_ARG(fpuset);
}

//------------------------------------------------------------------------------
void GridDriver::_start_execute_motion_hook(t_grid_state &gs,
                                            const t_fpuset &fpuset,
                                            const t_fpu_positions &initial_positions)
{
    // TODO

    // Temporary for now
    UNUSED_ARG(gs);
    UNUSED_ARG(fpuset);
    UNUSED_ARG(initial_positions);
}

//------------------------------------------------------------------------------
void GridDriver::_cancel_execute_motion_hook(t_grid_state &gs,
                                             const t_fpuset &fpuset,
                                             const t_fpu_positions &initial_positions)
{
    // TODO

    // Temporary for now
    UNUSED_ARG(gs);
    UNUSED_ARG(fpuset);
    UNUSED_ARG(initial_positions);
}

//------------------------------------------------------------------------------
void GridDriver::_post_execute_motion_hook(t_grid_state &gs,
                                           const t_grid_state &old_gs,
                                           const t_grid_state &move_gs,
                                           const t_fpuset &fpuset)
{
    // TODO

    // Temporary for now
    UNUSED_ARG(gs);
    UNUSED_ARG(old_gs);
    UNUSED_ARG(move_gs);
    UNUSED_ARG(fpuset);
}

//------------------------------------------------------------------------------
// TODO: Boost.Python wrapper test member function - remove eventually
double GridDriver::boostPythonDivide(double dividend, double divisor)
{
    return dividend / divisor;
}


//==============================================================================

} // namespace mpifps

