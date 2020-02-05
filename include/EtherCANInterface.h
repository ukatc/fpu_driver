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

#ifndef ETHER_CAN_INTERFACE_H
#define ETHER_CAN_INTERFACE_H

#include <stdio.h>
#include <unistd.h>
#include "InterfaceConstants.h"
#include "EtherCANInterfaceConfig.h"
#include "ethercan/time_utils.h"

#include "ethercan/AsyncInterface.h"

#if (__cplusplus < 201103L)
#error "Currently, this code requires C++11 support."
/* The requirement can certainly be loosened if needed. The most
   important C++11 feature is std::unique_ptr. */

#endif

namespace mpifps
{

using ethercanif::E_DATUM_TIMEOUT_FLAG;
using ethercanif::DATUM_TIMEOUT_ENABLE;
using ethercanif::DATUM_TIMEOUT_DISABLE;



class EtherCANInterface : public ethercanif::AsyncInterface
{
public:

    // number of retries for each action
    const int DEFAULT_NUM_RETRIES = 10;


    EtherCANInterface(const EtherCANInterfaceConfig config_values);

    ~EtherCANInterface()
    {
        t_grid_state grid_state;

        getGridState(grid_state); // throw away return value

        if (grid_state.interface_state == DS_CONNECTED)
        {
            LOG_CONTROL(LOG_INFO, "%18.6f : ~GridDriver(): disconnecting driver\n",
                        ethercanif::get_realtime());
            disconnect(); // throw away return value
        }

        // Flush the file descriptors for the CONTROL, TX and RX logs
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
    EtherCANInterface(const EtherCANInterface& other) = delete;
    // no assignment operator
    EtherCANInterface& operator=(const EtherCANInterface& other) = delete;
#if (__cplusplus >= 201103L)
    // no move constructor
    EtherCANInterface(EtherCANInterface&& other) = delete;
    // no move assignment operator
    EtherCANInterface& operator=(EtherCANInterface&& other) = delete;
#endif


    int getNumFPUs() const;

    E_EtherCANErrCode initializeGrid(t_grid_state& grid_state, t_fpuset const &fpuset);

    E_EtherCANErrCode resetFPUs(t_grid_state& grid_state, t_fpuset const &fpuset);

    E_EtherCANErrCode pingFPUs(t_grid_state& grid_state, t_fpuset const &fpuset);

    // find datum with automatic firmware operation
    E_EtherCANErrCode findDatum(t_grid_state& grid_state,
                                E_DATUM_SEARCH_DIRECTION * p_direction_flags=nullptr,
                                E_DATUM_SELECTION arm_selection=DASEL_BOTH,
                                E_DATUM_TIMEOUT_FLAG timeout_flag=DATUM_TIMEOUT_ENABLE,
                                bool count_protection=true,
                                t_fpuset const  * const fpuset=nullptr);

    E_EtherCANErrCode startFindDatum(t_grid_state& grid_state,
                                     E_DATUM_SEARCH_DIRECTION * p_direction_flags=nullptr,
                                     E_DATUM_SELECTION arm_selection=DASEL_BOTH,
                                     E_DATUM_TIMEOUT_FLAG timeout_flag=DATUM_TIMEOUT_ENABLE,
                                     bool count_protection=true,
                                     t_fpuset const * const fpuset=nullptr);

    E_EtherCANErrCode waitFindDatum(t_grid_state& grid_state,
                                    double &max_wait_time, bool &finished,
                                    t_fpuset const * const fpuset=nullptr);

    E_EtherCANErrCode configMotion(const t_wtable& waveforms,
                                   t_grid_state& grid_state,
                                   t_fpuset const &fpuset,
                                   bool allow_uninitialized=false,
                                   int ruleset_version=DEFAULT_WAVEFORM_RULESET_VERSION);

    E_EtherCANErrCode executeMotion(t_grid_state& grid_state, t_fpuset const &fpuset, bool sync_command=false);

    E_EtherCANErrCode startExecuteMotion(t_grid_state& grid_state, t_fpuset const &fpuset, bool sync_message=false);

    E_EtherCANErrCode waitExecuteMotion(t_grid_state& grid_state,
                                        double &max_wait_time,
                                        bool &finished,
                                        t_fpuset const &fpuset);

    E_EtherCANErrCode repeatMotion(t_grid_state& grid_state, t_fpuset const &fpuset);

    E_EtherCANErrCode reverseMotion(t_grid_state& grid_state, t_fpuset const &fpuset);

    E_EtherCANErrCode abortMotion(t_grid_state& grid_state, t_fpuset const &fpuset, bool sync_message=true);

    E_EtherCANErrCode freeBetaCollision(int fpu_id, E_REQUEST_DIRECTION request_dir,
                                        t_grid_state& grid_state);

    E_EtherCANErrCode enableBetaCollisionProtection(t_grid_state& grid_state);

    E_EtherCANErrCode setUStepLevel(int ustep_level, t_grid_state& grid_state, t_fpuset const &fpuset);

    E_EtherCANErrCode readRegister(uint16_t read_address,
                                   t_grid_state& grid_state,
                                   t_fpuset const &fpuset);

    E_EtherCANErrCode getFirmwareVersion(t_grid_state& grid_state, t_fpuset const &fpuset);


    E_EtherCANErrCode  getMinFirmwareVersion(t_fpuset const &fpuset,
            uint8_t (&min_firmware_version)[3],
            t_grid_state& grid_state);

    E_EtherCANErrCode lockFPU(int fpu_id, t_grid_state& grid_state);

    E_EtherCANErrCode unlockFPU(int fpu_id, t_grid_state& grid_state);

    E_EtherCANErrCode readSerialNumbers(t_grid_state& grid_state, t_fpuset const &fpuset);

    E_EtherCANErrCode writeSerialNumber(int fpu_id, const char serial_number[],
                                        t_grid_state& grid_state);

    E_EtherCANErrCode resetStepCounters(long alpha_steps, long beta_steps,
				       t_grid_state& grid_state, t_fpuset const &fpuset);

    E_EtherCANErrCode enableMove(int fpu_id, t_grid_state& grid_state);

    E_EtherCANErrCode enableAlphaLimitProtection(t_grid_state& grid_state);

    E_EtherCANErrCode freeAlphaLimitBreach(int fpu_id, E_REQUEST_DIRECTION request_dir,
                                           t_grid_state& grid_state);

    E_EtherCANErrCode setStepsPerSegment(int minsteps,
                                         int maxsteps,
                                         t_grid_state& grid_state,
                                         t_fpuset const &fpuset);

    // set number of 100ns clock ticks per waveform segment
    E_EtherCANErrCode setTicksPerSegment(unsigned long ticks,
                                         t_grid_state& grid_state,
                                         t_fpuset const &fpuset);

    E_EtherCANErrCode checkIntegrity(t_grid_state& grid_state,
                                     t_fpuset const &fpuset);

private:

    // this mutex ensures that no new
    // command is initiated while a running
    // command waits for completion.
    pthread_mutex_t command_creation_mutex = PTHREAD_MUTEX_INITIALIZER;



};

} // end of namespace
#endif
