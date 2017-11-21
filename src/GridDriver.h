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

#include "AsyncDriver.h"

namespace mpifps
{


class GridDriver : public AsyncDriver
{
public:

    // number of retries for each action
    const int DEFAULT_NUM_RETRIES = 10;

    GridDriver(int nfpus) : AsyncDriver(nfpus)
    {
        num_fpus = nfpus;
        memset((void*) &grid_state, 0, sizeof(grid_state));
    }

    ~GridDriver()
    {
    }

    E_DriverErrCode initializeGrid();

    E_DriverErrCode resetFPUs();

    E_DriverErrCode findDatum();

    E_DriverErrCode configMotion(const t_wtable& waveforms);

    E_DriverErrCode executeMotion();

    E_DriverErrCode repeatMotion();

    E_DriverErrCode reverseMotion();

    E_DriverErrCode abortMotion();

    E_DriverErrCode assignPositions();

    E_DriverErrCode lockFPU();

    E_DriverErrCode unlockFPU();


private:

    E_DriverErrCode untangleFPU();

    E_DriverErrCode clearCollision();

    int num_fpus;

    // this mutex ensures that no new
    // command is initiated while a running
    // command waits for completion.
    pthread_mutex_t command_creation_mutex = PTHREAD_MUTEX_INITIALIZER;


    // Buffer for retrieved state of the FPUs.
    t_grid_state grid_state;

};

} // end of namespace
#endif
