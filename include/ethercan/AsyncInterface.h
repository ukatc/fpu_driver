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

typedef struct
{
    int16_t alpha_steps;
    int16_t beta_steps;
} t_step_pair;

using t_waveform_steps = std::vector<t_step_pair>;

typedef struct
{
    int16_t fpu_id;
    t_waveform_steps steps;
} t_waveform;

// TODO: t_wtable is used as a variable-length array in the Python version of
// e.g. UnprotectedGridDriver::configMotion(), so might it be better to be a
// std::map<>?
typedef std::vector<t_waveform> t_wtable;

typedef bool t_fpuset[MAX_NUM_POSITIONERS];

typedef E_DATUM_SEARCH_DIRECTION t_datum_search_flags[MAX_NUM_POSITIONERS];


class AsyncInterface
{
public:
    /* Maximum number of retries to initialize configure
       motion before the driver will give up. */
    const int MAX_CONFIG_MOTION_RETRIES = 5;

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

    // get count of states across FPUs in a grid or sub-set of the grid.

    void getStateCount(const t_grid_state& grid_state, t_fpuset const * const pfpuset, t_counts &counts);

    E_EtherCANErrCode pingFPUsAsync(t_grid_state& grid_state, E_GridState& state_summary, t_fpuset const &fpuset);

    E_EtherCANErrCode resetFPUsAsync(t_grid_state& grid_state, E_GridState& state_summary, t_fpuset const &fpuset,
                                     const bool include_locked_fpus=false);

    E_EtherCANErrCode resetStepCountersAsync(long alpha_steps, long beta_steps,
					    t_grid_state& grid_state, E_GridState& state_summary, t_fpuset const &fpuset);

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
                                        bool allow_uninitialized=false,
                                        int ruleset_version=DEFAULT_WAVEFORM_RULESET_VERSION);

    E_EtherCANErrCode startExecuteMotionAsync(t_grid_state& grid_state, E_GridState& state_summary,
					      t_fpuset const &fpuset, bool sync_message=false);

    E_EtherCANErrCode waitExecuteMotionAsync(t_grid_state& grid_state,
            E_GridState& state_summary,
            double &max_wait_time,
            bool &finished,
            t_fpuset const &fpuset);

    E_EtherCANErrCode repeatMotionAsync(t_grid_state& grid_state, E_GridState& state_summary, t_fpuset const &fpuset);

    E_EtherCANErrCode reverseMotionAsync(t_grid_state& grid_state, E_GridState& state_summary, t_fpuset const &fpuset);

    E_EtherCANErrCode abortMotionAsync(pthread_mutex_t & command_mutex,
                                       t_grid_state& grid_state,
                                       E_GridState& state_summary,
                                       t_fpuset const &fpuset,
				       bool sync_message=true);

    E_EtherCANErrCode enableMoveAsync(int fpu_id,
                                      t_grid_state& grid_state,
                                      E_GridState& state_summary);

    E_EtherCANErrCode lockFPUAsync(int fpu_id, t_grid_state& grid_state, E_GridState& state_summary);

    E_EtherCANErrCode unlockFPUAsync(int fpu_id, t_grid_state& grid_state, E_GridState& state_summary);

    // retrieve cached minimum firmware version
    void getCachedMinFirmwareVersion(t_fpuset const &fpuset,
                                     bool &was_retrieved,
                                     uint8_t (&min_firmware_version)[3],
                                     int &min_firmware_fpu) const;

    // retrieve minimum firmware version over the network
    E_EtherCANErrCode getFirmwareVersionAsync(t_grid_state& grid_state, E_GridState& state_summary, t_fpuset const &fpuset);

    E_EtherCANErrCode enableBetaCollisionProtectionAsync(t_grid_state& grid_state,
            E_GridState& state_summary);

    E_EtherCANErrCode enableAlphaLimitProtectionAsync(t_grid_state& grid_state,
            E_GridState& state_summary);

    E_EtherCANErrCode freeBetaCollisionAsync(int fpu_id, E_REQUEST_DIRECTION request_dir,
            t_grid_state& grid_state,
            E_GridState& state_summary);

    E_EtherCANErrCode freeAlphaLimitBreachAsync(int fpu_id, E_REQUEST_DIRECTION request_dir,
            t_grid_state& grid_state,
            E_GridState& state_summary);

    E_EtherCANErrCode setUStepLevelAsync(int ustep_level,
                                         t_grid_state& grid_state,
                                         E_GridState& state_summary,
                                         t_fpuset const &fpuset);

    // set minimum and maximum number of steps per waveform segment
    // (the upper value is ignored for now)
    E_EtherCANErrCode setStepsPerSegmentAsync(int minsteps,
            int maxsteps,
            t_grid_state& grid_state,
            E_GridState& state_summary,
            t_fpuset const &fpuset);

    // set number of 100ns clock ticks per waveform segment
    E_EtherCANErrCode setTicksPerSegmentAsync(unsigned long ticks,
            t_grid_state& grid_state,
            E_GridState& state_summary,
            t_fpuset const &fpuset);

    E_EtherCANErrCode readRegisterAsync(uint16_t read_address,
                                        t_grid_state& grid_state,
                                        E_GridState& state_summary,
                                        t_fpuset const &fpuset);

    E_EtherCANErrCode checkIntegrityAsync(t_grid_state& grid_state,
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

    E_EtherCANErrCode validateWaveformsV4(const t_wtable& waveforms,
                                          const int MIN_STEPS,
                                          const int MAX_STEPS,
                                          const int MAX_START_STEPS,
                                          const unsigned int MAX_NUM_SECTIONS,
                                          const double MAX_INCREASE) const;

    E_EtherCANErrCode validateWaveformsV5(const t_wtable& waveforms,
                                          const int MIN_STEPS,
                                          const int MAX_STEPS,
                                          const int MAX_START_STEPS,
                                          const unsigned int MAX_NUM_SECTIONS,
                                          const int MAX_STEP_CHANGE) const;

    void logGridState(const E_LogLevel logLevel, t_grid_state& grid_state) const;

protected:

    const EtherCANInterfaceConfig config;
    unsigned int log_repeat_count;

    void getFPUsetOpt(t_fpuset const * const fpuset_opt, t_fpuset &fpuset) const;

    int countMoving(const t_grid_state &grid_state, t_fpuset const &fpuset) const;

    // make sure we have a certain minimum firmware version
    E_EtherCANErrCode assureMinFirmwareVersion(const int req_fw_major,
            const int req_fw_minor,
            const int req_fw_patch,
            const char* caller_name,
            t_fpuset const &fpuset,
            t_grid_state& grid_state);

    // retrieve minimum firmware version
    E_EtherCANErrCode getMinFirmwareVersionAsync(t_fpuset const &fpuset,
            uint8_t (&min_firmware_version)[3],
            int &min_firmware_fpu,
            t_grid_state& grid_state,
            E_GridState& state_summary);

    E_EtherCANErrCode readSerialNumbersAsync(t_grid_state& grid_state,
            E_GridState& state_summary, t_fpuset const &fpuset);

    E_EtherCANErrCode writeSerialNumberAsync(int fpu_id, const char serial_number[],
            t_grid_state& grid_state,
            E_GridState& state_summary);
private:

    int num_gateways;

    // cached firmware version of each FPU
    uint8_t fpu_firmware_version[MAX_NUM_POSITIONERS][3];

    GatewayInterface gateway;
#if CAN_PROTOCOL_VERSION == 1
    E_DATUM_SELECTION last_datum_arm_selection;
#endif

};

// some helper functions for logging

bool p_repeat_log(unsigned int &log_repeat_count);

const char * str_interface_state(const E_InterfaceState interface_state);

const char * str_fpu_state(const E_FPU_STATE state);

} // end of namespace

} // end of namespace
#endif
