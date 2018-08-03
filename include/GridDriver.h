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

#include <stdio.h>
#include <unistd.h>
#include "DriverConstants.h"
#include "GridDriverConfig.h"
#include "canlayer/time_utils.h"

#include "canlayer/AsyncDriver.h"

#if (__cplusplus < 201103L)
#error "Currently, this code requires C++11 support."
/* The requirement can certainly be loosened if needed. The most
   important C++11 feature is std::unique_ptr. */

#endif

namespace mpifps
{

using canlayer::E_DATUM_TIMEOUT_FLAG;
using canlayer::DATUM_TIMEOUT_ENABLE;
using canlayer::DATUM_TIMEOUT_DISABLE;



class GridDriver : public canlayer::AsyncDriver
{
public:

    // number of retries for each action
    const int DEFAULT_NUM_RETRIES = 10;


    GridDriver(const GridDriverConfig config_values)
        : AsyncDriver(config_values)
    {

        LOG_CONTROL(LOG_INFO, "%18.6f : starting driver for %i FPUs\n",
                    canlayer::get_realtime(), config.num_fpus);


    }

    ~GridDriver()
    {
        t_grid_state grid_state;

        getGridState(grid_state); // throw away return value

        if (grid_state.driver_state == DS_CONNECTED)
        {
            LOG_CONTROL(LOG_INFO, "%18.6f : ~GridDriver(): disconnecting driver\n",
                        canlayer::get_realtime());
            disconnect(); // throw away return value
        }

        if (config.fd_controllog >= 0)
        {
            syncfs(config.fd_controllog);
        }
        if (config.fd_txlog >= 0)
        {
            syncfs(config.fd_txlog);
        }
        if (config.fd_rxlog >= 0)
        {
            syncfs(config.fd_rxlog);
        }
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


    E_DriverErrCode initializeGrid(t_grid_state& grid_state, t_fpuset const &fpuset);

    E_DriverErrCode resetFPUs(t_grid_state& grid_state, t_fpuset const &fpuset);

    E_DriverErrCode pingFPUs(t_grid_state& grid_state, t_fpuset const &fpuset);

    // find datum with automatic firmware operation
    E_DriverErrCode findDatum(t_grid_state& grid_state,
                              E_DATUM_SEARCH_DIRECTION * p_direction_flags=nullptr,
                              E_DATUM_SELECTION arm_selection=DASEL_BOTH,
			      E_DATUM_TIMEOUT_FLAG timeout_flag=DATUM_TIMEOUT_ENABLE,
                              bool count_protection=true,
                              t_fpuset const  * const fpuset=nullptr);

    E_DriverErrCode startFindDatum(t_grid_state& grid_state,
                                   E_DATUM_SEARCH_DIRECTION * p_direction_flags=nullptr,
                                   E_DATUM_SELECTION arm_selection=DASEL_BOTH,
				   E_DATUM_TIMEOUT_FLAG timeout_flag=DATUM_TIMEOUT_ENABLE,
                                   bool count_protection=true,
                                   t_fpuset const * const fpuset=nullptr);

    E_DriverErrCode waitFindDatum(t_grid_state& grid_state,
                                  double &max_wait_time, bool &finished,
                                  t_fpuset const * const fpuset=nullptr);

    E_DriverErrCode configMotion(const t_wtable& waveforms,
                                 t_grid_state& grid_state,
                                 t_fpuset const &fpuset,
				 bool soft_protection=true,
                                 bool allow_uninitialized=false);

    E_DriverErrCode executeMotion(t_grid_state& grid_state, t_fpuset const &fpuset);

    E_DriverErrCode startExecuteMotion(t_grid_state& grid_state, t_fpuset const &fpuset);

    E_DriverErrCode waitExecuteMotion(t_grid_state& grid_state,
                                      double &max_wait_time,
                                      bool &finished,
                                      t_fpuset const &fpuset);


    E_DriverErrCode getPositions(t_grid_state& grid_state, t_fpuset const &fpuset);

    E_DriverErrCode getCounterDeviation(t_grid_state& grid_state, t_fpuset const &fpuset);

    E_DriverErrCode repeatMotion(t_grid_state& grid_state, t_fpuset const &fpuset);

    E_DriverErrCode reverseMotion(t_grid_state& grid_state, t_fpuset const &fpuset);

    E_DriverErrCode abortMotion(t_grid_state& grid_state, t_fpuset const &fpuset);

    E_DriverErrCode freeBetaCollision(int fpu_id, E_REQUEST_DIRECTION request_dir,
                                      t_grid_state& grid_state);

    E_DriverErrCode enableBetaCollisionProtection(t_grid_state& grid_state);

    E_DriverErrCode setUStepLevel(int ustep_level, t_grid_state& grid_state, t_fpuset const &fpuset);

    E_DriverErrCode readRegister(uint16_t read_address,
                                 t_grid_state& grid_state,
                                 t_fpuset const &fpuset);

    E_DriverErrCode getFirmwareVersion(t_grid_state& grid_state, t_fpuset const &fpuset);

    E_DriverErrCode lockFPU(int fpu_id, t_grid_state& grid_state);

    E_DriverErrCode unlockFPU(int fpu_id, t_grid_state& grid_state);

    E_DriverErrCode readSerialNumbers(t_grid_state& grid_state, t_fpuset const &fpuset);

    E_DriverErrCode writeSerialNumber(int fpu_id, const char serial_number[],
                                      t_grid_state& grid_state);

    int getNumFPUs() const;

private:

    E_DriverErrCode untangleFPU();

    E_DriverErrCode clearCollision();

    // this mutex ensures that no new
    // command is initiated while a running
    // command waits for completion.
    pthread_mutex_t command_creation_mutex = PTHREAD_MUTEX_INITIALIZER;



};

} // end of namespace
#endif
