// -*- mode: c++ -*-
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
// NAME GridDriver.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#ifndef GRID_DRIVER_H
#define GRID_DRIVER_H

#include "DriverConstants.h"
#include "canlayer/AsyncDriver.h"

#if (__cplusplus < 201103L)
#error "Currently, this code requires C++11 support."
/* The requirement can certainly be loosened if needed. The most
   important C++11 feature is std::unique_ptr. */

#endif

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

    // prohibit copying of the class - this would lead to resource leaks
    // and / or deadlocks
    // no copy constructor
    GridDriver(const GridDriver& other) = delete;
    // no assignment operator
    GridDriver& operator=(const GridDriver& other) = delete;
#if (__cplusplus >= 201103L)
    // no move constructor
    GridDriver(GridDriver&& other) = delete;
    // no move assignment operator
    GridDriver& operator=(GridDriver&& other) = delete;
#endif


    E_DriverErrCode initializeGrid(t_grid_state& grid_state);

    E_DriverErrCode resetFPUs(t_grid_state& grid_state);

    E_DriverErrCode pingFPUs(t_grid_state& grid_state);

    // find datum with automatic firmware operation
    E_DriverErrCode findDatum(t_grid_state& grid_state);

    E_DriverErrCode startFindDatum(t_grid_state& grid_state);

    E_DriverErrCode waitFindDatum(t_grid_state& grid_state,
                                  double &max_wait_time, bool &finished);

    E_DriverErrCode configMotion(const t_wtable& waveforms, t_grid_state& grid_state, bool check_protection);

    E_DriverErrCode executeMotion(t_grid_state& grid_state);

    E_DriverErrCode startExecuteMotion(t_grid_state& grid_state);

    E_DriverErrCode waitExecuteMotion(t_grid_state& grid_state, double &max_wait_time, bool &finished);


    E_DriverErrCode getPositions(t_grid_state& grid_state);

    E_DriverErrCode getCounterDeviation(t_grid_state& grid_state);

    E_DriverErrCode repeatMotion(t_grid_state& grid_state);

    E_DriverErrCode reverseMotion(t_grid_state& grid_state);

    E_DriverErrCode abortMotion(t_grid_state& grid_state);

    E_DriverErrCode freeBetaCollision(int fpu_id, E_REQUEST_DIRECTION request_dir,
                                      t_grid_state& grid_state);

    E_DriverErrCode enableBetaCollisionProtection(t_grid_state& grid_state);

    E_DriverErrCode setUStepLevel(int ustep_level, t_grid_state& grid_state);

    E_DriverErrCode lockFPU(t_grid_state& grid_state);

    E_DriverErrCode unlockFPU(t_grid_state& grid_state);

    int getNumFPUs() const;

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
