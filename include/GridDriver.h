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
// NAME GridDriver.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#ifndef GRID_DRIVER_H
#define GRID_DRIVER_H

#include "canlayer/AsyncDriver.h"

namespace mpifps
{


class GridDriver : public canlayer::AsyncDriver
{
public:

    // number of retries for each action
    const int DEFAULT_NUM_RETRIES = 10;

    GridDriver(int nfpus) : AsyncDriver(nfpus)
    {
        num_fpus = nfpus;
    }

    ~GridDriver()
    {
    }

    E_DriverErrCode initializeGrid(t_grid_state& grid_state);

    E_DriverErrCode resetFPUs(t_grid_state& grid_state);

    // find datum with driver-controlled movement
    E_DriverErrCode findDatum(t_grid_state& grid_state);

    // find datum with automatic firmware operation
    E_DriverErrCode autoFindDatum(t_grid_state& grid_state);

    E_DriverErrCode configMotion(const t_wtable& waveforms, t_grid_state& grid_state);

    E_DriverErrCode executeMotion(t_grid_state& grid_state);

    E_DriverErrCode getPositions(t_grid_state& grid_state);

    E_DriverErrCode repeatMotion(t_grid_state& grid_state);

    E_DriverErrCode reverseMotion(t_grid_state& grid_state);

    E_DriverErrCode abortMotion(t_grid_state& grid_state);

    E_DriverErrCode assignPositions(t_grid_state& grid_state);

    E_DriverErrCode lockFPU(t_grid_state& grid_state);

    E_DriverErrCode unlockFPU(t_grid_state& grid_state);

    

private:

    E_DriverErrCode untangleFPU();

    E_DriverErrCode clearCollision();

    int num_fpus;

    // this mutex ensures that no new
    // command is initiated while a running
    // command waits for completion.
    pthread_mutex_t command_creation_mutex = PTHREAD_MUTEX_INITIALIZER;



};

} // end of namespace
#endif
