// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-11-03  Created (translated from Python fpu_commands.py).
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME FPUCommands.C
//
// Various utility functions.
//
////////////////////////////////////////////////////////////////////////////////

#include <limits>
#include <string>
#include "FPUCommands.h"

namespace mpifps
{

//------------------------------------------------------------------------------
void list_angles(const t_grid_state &gs, int num_fpus, 
                 t_fpus_angles &fpus_angles_ret, double alpha_datum_offset,
                 bool show_uninitialized, double asteps_per_deg,
                 double bsteps_per_deg)
{
    // Determines approximate angular positions for each FPU in the grid. The
    // optional asteps_per_deg and bsteps_per_deg arguments are the scaling
    // factors.

    // *** Lambda ***
    auto tvalid = [&](bool zeroed)
    {
        if (zeroed || show_uninitialized)
        {
            return 1.0;
        }
        else
        {
            return std::numeric_limits<double>::quiet_NaN();
        }
    };

    fpus_angles_ret.clear();
    for (int fpu_id = 0; fpu_id < num_fpus; fpu_id++)
    {
        const t_fpu_state &fpu_state = gs.FPU_state[fpu_id];
        double alpha_angle = 
            ((((double)fpu_state.alpha_steps) / asteps_per_deg) + alpha_datum_offset) *
            tvalid(fpu_state.alpha_was_referenced);
        double beta_angle =
            (((double)fpu_state.beta_steps) / bsteps_per_deg) *
            tvalid(fpu_state.beta_was_referenced);
        
        fpus_angles_ret.push_back({fpu_id, {alpha_angle, beta_angle}});
    }
}

//------------------------------------------------------------------------------
std::string doubleToString(double val)
{
    // Converts a double value to a string, with a sensible number of decimal
    // places.

    // TODO: Finish this function so that displays sensible number of decimal
    // places - e.g. use std::stringstream's?
    return std::to_string(val);
}

//------------------------------------------------------------------------------

} // namespace mpifps

