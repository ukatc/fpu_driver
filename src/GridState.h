// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
// ESO - VLT Project
//
// Copyright 2017 E.S.O,
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client sample
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME T_GridState.h
//
// This header defines a struct which holds the mirrored state
// if the whole FPU array
//
////////////////////////////////////////////////////////////////////////////////

#ifndef GRID_STATE_H
#define GRID_STATE_H

#include "T_GridState.h"

namespace mpifps
{

    E_GridState getGridStateSummary(const FPUGridState& grid_state);

}
