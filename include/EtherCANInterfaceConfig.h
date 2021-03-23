// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME EtherCANInterfaceConfig.h
//
// This file defines a POD structure with pre-set configuration values
// TODO: BW: This structure isn't POD because it has a constructor (and now also
// contains a std::vector), so change the above comment? BUT also check that it
// doesn't actually need to be POD.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef GRID_INTERFACE_CONFIG_H
#define GRID_INTERFACE_CONFIG_H

#include <vector>
#include "E_LogLevel.h"
#include "InterfaceConstants.h"
#include "FPUConstants.h"
#ifdef FLEXIBLE_CAN_MAPPING
#include "ErrorCodes.h"
#endif // FLEXIBLE_CAN_MAPPING

namespace mpifps
{

// TODO: Move t_fpuset definition and clearFpuSet() to a different file 
// eventually?
typedef bool t_fpuset[MAX_NUM_POSITIONERS];

void clearFpuSet(t_fpuset &fpuset_to_clear);


//==============================================================================
struct EtherCANInterfaceConfig
{
public:

    // TCP connection parameters. This sets the keepalive option which
    // helps to detect a failed connection.

    // Connection time-out value. If set to zero, using of keep-alive
    // packets is disabled.
    double SocketTimeOutSeconds;

    // idle time before keep-alive probes are sent
    int TCP_IdleSeconds;

    // interval with which keep-alive packets are sent
    int TCP_KeepaliveIntervalSeconds;

    // Current logging level (see E_LogLevel.h)
    E_LogLevel logLevel;

    // File descriptor for log of commands and results in the control context
    // (CONTROL)
    int fd_controllog;

    // File descriptor for log of all transmitted CAN commands (TX)
    int fd_txlog;

    // File descriptor for log of all received CAN responses (RX)
    int fd_rxlog;

#ifndef FLEXIBLE_CAN_MAPPING // NOT FLEXIBLE_CAN_MAPPING
    int num_fpus;
#endif // NOT FLEXIBLE_CAN_MAPPING

    // Offset with which alpha arm angles are computed from step counts
    double alpha_datum_offset;

    double motor_minimum_frequency;   // lower bound of stepper motor frequency
    double motor_maximum_frequency;   // upper bound of stepper motor frequency
    double motor_max_start_frequency; // maximum start frequency
    double motor_max_rel_increase;    // maximum frequency growth factor. Used in ruleset v1-v4.
    int motor_max_step_difference;    /* maximum difference in number of steps between segments
                                         (which allows for a constant acceleration).
					 Used in ruleset V5. */

    // Waveform upload parameters
    long waveform_upload_pause_us; // wait time before a new waveform step is sent to the same FPU
    bool confirm_each_step; // request confirmation for each waveform step

    int min_bus_repeat_delay_ms; // delay, in milliseconds, before writing to the same CAN bus
    int min_fpu_repeat_delay_ms; // delay, in milliseconds, before writing to the same FPU

    int firmware_version_address_offset;
    int configmotion_confirmation_period;
    int can_command_priority; // maximum priority of CAN commands; this is a four-bit value
    int configmotion_max_retry_count; // number of times time-outs
				      // will be reported and missing
				      // data is send again.
    int configmotion_max_resend_count; // number of times all data
				       // will be resent silently on a
				       // low level.

    //--------------------------------------------------------------------------
    EtherCANInterfaceConfig()
        : logLevel(LOG_TRACE_CAN_MESSAGES)
    {
#ifdef FLEXIBLE_CAN_MAPPING
        clearFpuSet(fpuset);
#else  // NOT FLEXIBLE_CAN_MAPPING
        num_fpus = MAX_NUM_POSITIONERS;
#endif // NOT FLEXIBLE_CAN_MAPPING

        // Set default time-out values
        SocketTimeOutSeconds = SOCKET_TIMEOUT_SECS;
        TCP_IdleSeconds = 10;
        TCP_KeepaliveIntervalSeconds = 1;

        waveform_upload_pause_us = 0;
        confirm_each_step = true;
        configmotion_confirmation_period = 25;
        can_command_priority = 3;

        min_bus_repeat_delay_ms = 2;
        min_fpu_repeat_delay_ms = 4;
        configmotion_max_retry_count = 10;
        configmotion_max_resend_count = 5;

        // New offset for v1.3.0, matching firmware version 1.4.4
        firmware_version_address_offset = 0x61;

        // Initialize log file descriptors
        fd_controllog = -1;
        fd_rxlog = -1;
        fd_txlog = -1;

        alpha_datum_offset = ALPHA_DATUM_OFFSET;
        motor_minimum_frequency = MOTOR_MIN_STEP_FREQUENCY;
        motor_maximum_frequency = MOTOR_MAX_STEP_FREQUENCY;
        motor_max_start_frequency = MOTOR_MAX_START_FREQUENCY;
        motor_max_rel_increase = MAX_ACCELERATION_FACTOR;
        motor_max_step_difference = 100;
    };

    //--------------------------------------------------------------------------

#ifdef FLEXIBLE_CAN_MAPPING
    E_EtherCANErrCode initFpuIdList(const std::vector<int> &fpu_id_list_init);
    bool isValidFpuId(int fpu_id) const;
    const std::vector<int> &getFpuIdList() const;
    const t_fpuset &getFpuSet() const;

private:
    std::vector<int> fpu_id_list;
    t_fpuset fpuset;

#endif // FLEXIBLE_CAN_MAPPING

};

//==============================================================================

}
#endif
