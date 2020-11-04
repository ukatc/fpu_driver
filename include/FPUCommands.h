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
// NAME FPUCommands.h
//
// TODO: Put description here
//
////////////////////////////////////////////////////////////////////////////////

#ifndef FPUCOMMANDS_H
#define FPUCOMMANDS_H

#include <vector>
#include <utility>
#include "FPUConstants.h"
#include "T_GridState.h"

namespace mpifps
{

typedef struct
{
    double alpha;
    double beta;
} t_fpu_angles;
using t_fpus_angles = std::vector<std::pair<int, t_fpu_angles>>;

void list_angles(const t_grid_state &gs, int num_fpus,
                 t_fpus_angles &fpus_angles_ret,
                 double alpha_datum_offset = -180.0,
                 bool show_uninitialized = false,
                 double asteps_per_deg = StepsPerDegreeAlpha,
                 double bsteps_per_deg = StepsPerDegreeBeta);

} // namespace mpifps

#endif // FPUCOMMANDS_H