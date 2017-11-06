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

    AsyncDriver()
        {
        }

    ~AsyncDriver()
        {
        }
    
    E_DriverErrCode initializeGridAsync();

    E_DriverErrCode resetFPUsAsync();

    E_DriverErrCode findDatumAsync();

    E_DriverErrCode configMotionAsync();

    E_DriverErrCode executeMotionAsync();

    E_DriverErrCode repeatMotionAsync();

    E_DriverErrCode reverseMotionAsync();

    E_DriverErrCode abortMotionAsync();

    E_DriverErrCode assignPositionsAsync();

    E_DriverErrCode lockFPUAsync();

    E_DriverErrCode unlockFPUAsync();

    void getGridState(t_grid_state& out_state);

    E_GridState waitForState(E_WaitTarget target, t_grid_state& out_detailed_state);
    
    
private:

    GatewayDriver gateway;
}

} // end of namespace
