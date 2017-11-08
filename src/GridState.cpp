////////////////////////////////////////////////////////////////////////////////
// ESO - VLT Project
//
// Copyright 2017 E.S.O,
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// Pablo Gutierrez 2017-07-22  created CAN client sample
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client sample
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME GridState.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#include "GridState.h"

namespace mpifps
{

E_GridState getGridStateSummary(const FPUGridState& grid_state)
{
    // get the summary state of the grid.
    // (This relies on that all FPU updates
    // do mirror the global counters correctly).
    
    // we simply ignored the locked units

    // this computation returns the "minimum operational state"
    // of all FPUs.
    // Note that his requires some ordering which is
    // in part arbitray; the testes are carried out in that order.

    // the high-priority error conditions are checked first
    if (grid_state.Counts[FPST_ABORTED] > 0)
    {
        return GS_ABORTED;
    }

    if (grid_state.Counts[FPST_COLLISION_DETECTED] > 0)
    {
        return GS_COLLISION;
    }

    if (grid_state.Counts[FPST_LIMIT_STOP] > 0)
    {
        return GS_LIMITSTOP;
    }

    // now we check in order of operational states
    if ((grid_state.Counts[FPST_UNINITIALISED] - grid_state.Counts[LOCKED]) > 0)
    {
        return GS_UNINITIALISED:
    }

    if (grid_state.Counts[FPST_COORDINATE_RECOVERY] > 0)
    {
        // results in the same: we don't know all
        // the coordinates in the grid
        return GS_UNINITIALISED;
    }

    if (grid_state.Counts[FPST_LEAVING_DATUM] > 0)
    {
        return GS_LEAVING_DATUM;
    }

    if (grid_state.Counts[FPST_ABOVE_DATUM] > 0)
    {
        return GS_ABOVE_DATUM;
    }

    if (grid_state.Counts[FPST_DATUM_SEARCH] > 0)
    {
        return GS_DATUM_SEARCH;
    }

    if (grid_state.Counts[FPST_INITIALISED] > 0)
    {
        return GS_INITIALISED;
    }

    if (grid_state.Counts[FPST_LOADING] > 0)
    {
        return GS_LOADING;
    }

    if (grid_state.Counts[FPST_READY_FORWARD] > 0)
    {
        return GS_READY_FORWARD;
    }

    if (grid_state.Counts[FPST_READY_BACKWARD] > 0)
    {
        return GS_READY_BACKWARD;
    }

    if (grid_state.Counts[FPST_MOVING] > 0)
    {
        return GS_MOVING;
    }

    return FINISHED;
    
}

}
