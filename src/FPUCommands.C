// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-11-03  TODO: Put comment here
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME FPUCommands.C
//
// TODO: Put description here
//
////////////////////////////////////////////////////////////////////////////////

#include <limits>
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
    // optional second and third argument are the scaling factors, and the
    // fourth argument is the number of FPUs shown.

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

} // namespace mpifps

