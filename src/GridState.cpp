////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
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
#include "DriverConstants.h"


namespace mpifps
{

E_GridState getGridStateSummary(const t_grid_state& grid_state)
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

    if (grid_state.Counts[FPST_OBSTACLE_ERROR] > 0)
    {
        return GS_COLLISION;
    }

    // now we check in order of operational states
    if ((grid_state.Counts[FPST_UNINITIALIZED] - grid_state.Counts[FPST_LOCKED]) > 0)
    {
        return GS_UNINITIALIZED;
    }


    if (grid_state.Counts[FPST_DATUM_SEARCH] > 0)
    {
        return GS_DATUM_SEARCH;
    }

    if (grid_state.Counts[FPST_AT_DATUM] > 0)
    {
        return GS_AT_DATUM;
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

    return GS_FINISHED;

}

}

