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

#ifndef ASYNC_DRIVER_H
#define ASYNC_DRIVER_H


#include "GatewayDriver.h"

namespace mpifps
{

namespace canlayer
{

class AsyncDriver
{
    public:

    typedef struct
    {
        int16_t alpha_steps;
        int16_t beta_steps;
    } t_step_pair;

    typedef struct
    {
        int16_t fpu_id;
        std::vector<t_step_pair> steps;
    } t_waveform;

    typedef  std::vector<t_waveform> t_wtable;

    AsyncDriver(int nfpus)
        : gateway(nfpus)
        {
            num_fpus = nfpus;
        }

    ~AsyncDriver()
        {
        }

    // Initialize internal data structures, allocate memory etc.
    // (this can fail if the system is too low on memory).
    E_DriverErrCode initializeDriver();


    // connect to gateways
    E_DriverErrCode connect(const int ngateways, const t_gateway_address gateway_addresses[]);

    // disconnect sockets, and re-add any pending commands to
    // the command queue. (This does not delete the
    // available status information about the FPUs,
    // but disables status updates).
    E_DriverErrCode disconnect();


    E_DriverErrCode initializeGridAsync(t_grid_state& grid_state, E_GridState& state_summary);

    E_DriverErrCode resetFPUsAsync(t_grid_state& grid_state, E_GridState& state_summary);

    E_DriverErrCode findDatumAsync(t_grid_state& grid_state, E_GridState& state_summary);

    E_DriverErrCode autoFindDatumAsync(t_grid_state& grid_state, E_GridState& state_summary);
    
    E_DriverErrCode configMotionAsync(t_grid_state& grid_state, E_GridState& state_summary, const t_wtable& waveforms);

    E_DriverErrCode executeMotionAsync(t_grid_state& grid_state, E_GridState& state_summary);

    E_DriverErrCode repeatMotionAsync(t_grid_state& grid_state, E_GridState& state_summary);

    E_DriverErrCode reverseMotionAsync(t_grid_state& grid_state, E_GridState& state_summary);

    E_DriverErrCode abortMotionAsync(t_grid_state& grid_state, E_GridState& state_summary);

    E_DriverErrCode assignPositionsAsync(t_grid_state& grid_state, E_GridState& state_summary);

    E_DriverErrCode lockFPUAsync(t_grid_state& grid_state, E_GridState& state_summary);

    E_DriverErrCode unlockFPUAsync(t_grid_state& grid_state, E_GridState& state_summary);

    E_GridState getGridState(t_grid_state& out_state);

    E_GridState waitForState(E_WaitTarget target,
                             t_grid_state& out_detailed_state);


    private:

    int num_fpus;
    GatewayDriver gateway;
};

} // end of namespace

} // end of namespace
#endif
