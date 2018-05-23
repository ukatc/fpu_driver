// -*- mode: c++ -*-
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 UKRI. See file "LICENSE" for license information.
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
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

#include "../GridDriverConfig.h"
#include "GatewayDriver.h"
#include "../DriverConstants.h"

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

    /* Maximum number of retries to initialize configure
       motion before the driver will give up. */
    const int MAX_CONFIG_MOTION_RETRIES = 5;

    AsyncDriver(const GridDriverConfig config_vals)
        : config(config_vals), gateway(config_vals)
    {
        num_gateways = 0;
        log_repeat_count = 0;

#if CAN_PROTOCOL_VERSION == 1
        // initialize field which records last arm selection
        last_datum_arm_selection = DASEL_NONE;
#endif
    }

    ~AsyncDriver()
    {
        if ( gateway.getDriverState() == DS_CONNECTED)
        {
            disconnect();
        }
        if ( gateway.getDriverState() != DS_UNINITIALIZED)
        {
            deInitializeDriver();
        }

    }

    // Initialize internal data structures, allocate memory etc.
    // (this can fail if the system is too low on memory).
    E_DriverErrCode initializeDriver();

    // deinitialize internal data structures..
    E_DriverErrCode deInitializeDriver();


    // connect to gateways
    E_DriverErrCode connect(const int ngateways, const t_gateway_address gateway_addresses[]);

    // disconnect sockets, and re-add any pending commands to
    // the command queue. (This does not delete the
    // available status information about the FPUs,
    // but disables status updates).
    E_DriverErrCode disconnect();


    E_DriverErrCode initializeGridAsync(t_grid_state& grid_state, E_GridState& state_summary);
    
    E_DriverErrCode getFirmwareVersionAsync(t_grid_state& grid_state, E_GridState& state_summary);

    E_DriverErrCode pingFPUsAsync(t_grid_state& grid_state, E_GridState& state_summary);

    E_DriverErrCode resetFPUsAsync(t_grid_state& grid_state, E_GridState& state_summary);

    E_DriverErrCode startAutoFindDatumAsync(t_grid_state& grid_state, E_GridState& state_summary,
                                            E_DATUM_SELECTION arm_selection);

    E_DriverErrCode waitAutoFindDatumAsync(t_grid_state& grid_state, E_GridState& state_summary,
                                           double &max_wait_time, bool &finished);

    E_DriverErrCode configMotionAsync(t_grid_state& grid_state, E_GridState& state_summary,
                                      const t_wtable& waveforms,  bool check_protection);

    E_DriverErrCode startExecuteMotionAsync(t_grid_state& grid_state, E_GridState& state_summary);

    E_DriverErrCode waitExecuteMotionAsync(t_grid_state& grid_state, E_GridState& state_summary, double &max_wait_time, bool &finished);

    E_DriverErrCode getPositionsAsync(t_grid_state& grid_state,
                                      E_GridState& state_summary);

    E_DriverErrCode getCounterDeviationAsync(t_grid_state& grid_state,
            E_GridState& state_summary);

    E_DriverErrCode repeatMotionAsync(t_grid_state& grid_state, E_GridState& state_summary);

    E_DriverErrCode reverseMotionAsync(t_grid_state& grid_state, E_GridState& state_summary);

    E_DriverErrCode abortMotionAsync(pthread_mutex_t & command_mutex, t_grid_state& grid_state, E_GridState& state_summary);

    E_DriverErrCode lockFPUAsync(t_grid_state& grid_state, E_GridState& state_summary);

    E_DriverErrCode unlockFPUAsync(t_grid_state& grid_state, E_GridState& state_summary);

    E_DriverErrCode enableBetaCollisionProtectionAsync(t_grid_state& grid_state,
            E_GridState& state_summary);

    E_DriverErrCode freeBetaCollisionAsync(int fpu_id, E_REQUEST_DIRECTION request_dir,
                                           t_grid_state& grid_state,
                                           E_GridState& state_summary);

    E_DriverErrCode setUStepLevelAsync(int ustep_level,
                                       t_grid_state& grid_state,
                                       E_GridState& state_summary);

    E_DriverErrCode readRegisterAsync(uint16_t read_address, t_grid_state& grid_state,
                                      E_GridState& state_summary);


    E_GridState getGridState(t_grid_state& out_state) const;

    E_GridState waitForState(E_WaitTarget target,
                             t_grid_state& out_detailed_state, double &max_wait_time, bool &cancelled) const;

    E_DriverErrCode validateWaveforms(const t_wtable& waveforms, const int MIN_STEPS, const int MAX_STEPS,
                                      const unsigned int MAX_NUM_SECTIONS, const double MAX_INCREASE) const;

    void logGridState(const E_LogLevel logLevel, t_grid_state& grid_state) const;

protected:

    const GridDriverConfig config;
    unsigned int log_repeat_count;


private:

    int num_gateways;
    GatewayDriver gateway;
#if CAN_PROTOCOL_VERSION == 1
    E_DATUM_SELECTION last_datum_arm_selection;
#endif

};

} // end of namespace

} // end of namespace
#endif
