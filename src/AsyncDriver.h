// -*- mode: c++ -*-
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
// NAME AsyncDriver.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#include "GatewayDriver.hpp"

namespace mpifps
{


class AsyncDriver
{
public:

    typedef struct
    {
        int16 alpha_steps;
        int16 beta_steps;
    } t_step_pair;
    
    typedef struct t_waveform
    {
        int fpu_id;
        std::vector<t_step_pair> steps;
    } t_wtable;
    
    AsyncDriver()
        {
        }

    ~AsyncDriver()
        {
        }
    
    E_DriverErrCode initializeGridAsync(t_grid_state& grid_state);

    E_DriverErrCode resetFPUsAsync(t_grid_state& grid_state);

    E_DriverErrCode findDatumAsync(t_grid_state& grid_state);

    E_DriverErrCode configMotionAsync(const t_wtable& waveforms);

    E_DriverErrCode executeMotionAsync(t_grid_state& grid_state);

    E_DriverErrCode repeatMotionAsync(t_grid_state& grid_state);

    E_DriverErrCode reverseMotionAsync(t_grid_state& grid_state);

    E_DriverErrCode abortMotionAsync(t_grid_state& grid_state);

    E_DriverErrCode assignPositionsAsync(t_grid_state& grid_state);

    E_DriverErrCode lockFPUAsync(t_grid_state& grid_state);

    E_DriverErrCode unlockFPUAsync(t_grid_state& grid_state);

    void getGridState(t_grid_state& out_state);

    E_GridState waitForState(E_WaitTarget target, t_grid_state& out_detailed_state);
    
    
private:

    GatewayDriver gateway;
}

} // end of namespace
