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

#ifndef ASYNC_INTERFACE_H
#define ASYNC_INTERFACE_H

#include <cmath>
#include "../EtherCANInterfaceConfig.h"
#include "GatewayInterface.h"
#include "../InterfaceConstants.h"
#include "E_CAN_COMMAND.h"

namespace mpifps
{

namespace ethercanif
{

class AsyncInterface
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

    typedef bool t_fpuset[MAX_NUM_POSITIONERS];

    typedef E_DATUM_SEARCH_DIRECTION t_datum_search_flags[MAX_NUM_POSITIONERS];

    /* Maximum number of retries to initialize configure
       motion before the driver will give up. */
    const int MAX_CONFIG_MOTION_RETRIES = 5;

    const uint8_t FIRMWARE_NOT_RETRIEVED = 0xff;

    explicit AsyncInterface(const EtherCANInterfaceConfig &config_vals)
        : config(config_vals), gateway(config_vals)
    {
        num_gateways = 0;
        log_repeat_count = 0;

        // initialize known firmware versions to zero
        memset(fpu_firmware_version, FIRMWARE_NOT_RETRIEVED, sizeof(fpu_firmware_version));

#if CAN_PROTOCOL_VERSION == 1
        // initialize field which records last arm selection
        last_datum_arm_selection = DASEL_NONE;
#endif
    }

    ~AsyncInterface()
    {
        if ( gateway.getInterfaceState() == DS_CONNECTED)
        {
            disconnect();
        }
        if ( gateway.getInterfaceState() != DS_UNINITIALIZED)
        {
            deInitializeInterface();
        }

    }

    // Initialize internal data structures, allocate memory etc.
    // (this can fail if the system is too low on memory).
    E_EtherCANErrCode initializeInterface();

    // deinitialize internal data structures..
    E_EtherCANErrCode deInitializeInterface();


    // connect to gateways
    E_EtherCANErrCode connect(const int ngateways, const t_gateway_address gateway_addresses[]);

    // disconnect sockets, and re-add any pending commands to
    // the command queue. (This does not delete the
    // available status information about the FPUs,
    // but disables status updates).
    E_EtherCANErrCode disconnect();


    E_EtherCANErrCode initializeGridAsync(t_grid_state& grid_state, E_GridState& state_summary, t_fpuset const &fpuset);

    E_EtherCANErrCode getFirmwareVersionAsync(t_grid_state& grid_state, E_GridState& state_summary, t_fpuset const &fpuset);

    E_EtherCANErrCode pingFPUsAsync(t_grid_state& grid_state, E_GridState& state_summary, t_fpuset const &fpuset);

    E_EtherCANErrCode resetFPUsAsync(t_grid_state& grid_state, E_GridState& state_summary, t_fpuset const &fpuset);

    E_EtherCANErrCode startAutoFindDatumAsync(t_grid_state& grid_state, E_GridState& state_summary,
            E_DATUM_SEARCH_DIRECTION * p_direction_flags=nullptr,
            E_DATUM_SELECTION arm_selection=DASEL_BOTH,
            E_DATUM_TIMEOUT_FLAG timeout_flag=DATUM_TIMEOUT_ENABLE,
            bool count_protection=true,
            t_fpuset const * const fpuset_opt=nullptr);

    E_EtherCANErrCode waitAutoFindDatumAsync(t_grid_state& grid_state,
            E_GridState& state_summary,
            double &max_wait_time,
            bool &finished,
            t_fpuset const  * const fpuset_opt=nullptr);

    E_EtherCANErrCode configMotionAsync(t_grid_state& grid_state,
                                        E_GridState& state_summary,
                                        const t_wtable& waveforms,
                                        t_fpuset const &fpuset,
                                        bool soft_protection=true,
                                        bool allow_uninitialized=false,
                                        int ruleset_version=DEFAULT_WAVEFORM_RULSET_VERSION);

    E_EtherCANErrCode startExecuteMotionAsync(t_grid_state& grid_state, E_GridState& state_summary, t_fpuset const &fpuset);

    E_EtherCANErrCode waitExecuteMotionAsync(t_grid_state& grid_state,
            E_GridState& state_summary,
            double &max_wait_time,
            bool &finished,
            t_fpuset const &fpuset);

    E_EtherCANErrCode getPositionsAsync(t_grid_state& grid_state,
                                        E_GridState& state_summary,
                                        t_fpuset const &fpuset);

    E_EtherCANErrCode getCounterDeviationAsync(t_grid_state& grid_state,
            E_GridState& state_summary,
            t_fpuset const &fpuset);

    E_EtherCANErrCode repeatMotionAsync(t_grid_state& grid_state, E_GridState& state_summary, t_fpuset const &fpuset);

    E_EtherCANErrCode reverseMotionAsync(t_grid_state& grid_state, E_GridState& state_summary, t_fpuset const &fpuset);

    E_EtherCANErrCode abortMotionAsync(pthread_mutex_t & command_mutex,
                                       t_grid_state& grid_state,
                                       E_GridState& state_summary,
                                       t_fpuset const &fpuset);

    E_EtherCANErrCode lockFPUAsync(int fpu_id, t_grid_state& grid_state, E_GridState& state_summary);

    E_EtherCANErrCode unlockFPUAsync(int fpu_id, t_grid_state& grid_state, E_GridState& state_summary);

    E_EtherCANErrCode enableBetaCollisionProtectionAsync(t_grid_state& grid_state,
            E_GridState& state_summary);

    E_EtherCANErrCode freeBetaCollisionAsync(int fpu_id, E_REQUEST_DIRECTION request_dir,
            t_grid_state& grid_state,
            E_GridState& state_summary);

    E_EtherCANErrCode setUStepLevelAsync(int ustep_level,
                                         t_grid_state& grid_state,
                                         E_GridState& state_summary,
                                         t_fpuset const &fpuset);

    E_EtherCANErrCode readRegisterAsync(uint16_t read_address,
                                        t_grid_state& grid_state,
                                        E_GridState& state_summary,
                                        t_fpuset const &fpuset);


    E_GridState getGridState(t_grid_state& out_state) const;

    E_GridState waitForState(E_WaitTarget target,
                             t_grid_state& out_detailed_state,
                             double &max_wait_time,
                             bool &cancelled) const;

    E_EtherCANErrCode validateWaveformsV1(const t_wtable& waveforms,
                                          const int MIN_STEPS,
                                          const int MAX_STEPS,
                                          const int MAX_START_STEPS,
                                          const unsigned int MAX_NUM_SECTIONS,
                                          const double MAX_INCREASE) const;

    E_EtherCANErrCode validateWaveformsV2(const t_wtable& waveforms,
                                          const int MIN_STEPS,
                                          const int MAX_STEPS,
                                          const int MAX_START_STEPS,
                                          const unsigned int MAX_NUM_SECTIONS,
                                          const double MAX_INCREASE) const;

    E_EtherCANErrCode validateWaveformsV3(const t_wtable& waveforms,
                                          const int MIN_STEPS,
                                          const int MAX_STEPS,
                                          const int MAX_START_STEPS,
                                          const unsigned int MAX_NUM_SECTIONS,
                                          const double MAX_INCREASE) const;

    void logGridState(const E_LogLevel logLevel, t_grid_state& grid_state) const;

protected:

    const EtherCANInterfaceConfig config;
    unsigned int log_repeat_count;

    void getFPUsetOpt(t_fpuset const * const fpuset_opt, t_fpuset &fpuset) const;

    int countMoving(const t_grid_state &grid_state, t_fpuset const &fpuset) const;

    E_EtherCANErrCode getMinFirmwareVersion(t_fpuset const &fpuset,
                                            uint8_t (&min_firmware_version)[3],
                                            int &min_firmware_fpu,
                                            t_grid_state& grid_state,
                                            E_GridState& state_summary);

    void getCachedMinFirmwareVersion(t_fpuset const &fpuset,
                                     bool &was_retrieved,
                                     uint8_t (&min_firmware_version)[3],
                                     int &min_firmware_fpu) const;

    E_EtherCANErrCode readSerialNumbersAsync(t_grid_state& grid_state,
            E_GridState& state_summary, t_fpuset const &fpuset);

    E_EtherCANErrCode writeSerialNumberAsync(int fpu_id, const char serial_number[],
            t_grid_state& grid_state,
            E_GridState& state_summary);
private:

    int num_gateways;
    // firmware version of each FPU, this is a work-around for protocol 1
    uint8_t fpu_firmware_version[MAX_NUM_POSITIONERS][3];
    GatewayInterface gateway;
#if CAN_PROTOCOL_VERSION == 1
    E_DATUM_SELECTION last_datum_arm_selection;
#endif

};

} // end of namespace

} // end of namespace
#endif