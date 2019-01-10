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
//
//
////////////////////////////////////////////////////////////////////////////////

#ifndef GRID_INTERFACE_CONFIG_H
#define GRID_INTERFACE_CONFIG_H

#include "E_LogLevel.h"
#include "InterfaceConstants.h"


namespace mpifps
{

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

    E_LogLevel logLevel;
    // file descriptor for log of commands and results in the control context
    int fd_controllog;

    // file descriptor for log of all sent CAN commands
    int fd_txlog;

    //file descriptor for log of all received CAN responses
    int fd_rxlog;

    int num_fpus;

    // offset with which alpha arm angles are computed from step counts
    double alpha_datum_offset;

    double motor_minimum_frequency;   // lower bound of stepper motor frequency
    double motor_maximum_frequency;   // upper bound of stepper motor frequency
    double motor_max_start_frequency; // maximum start frequency
    double motor_max_rel_increase;    // maximum frequency growth factor

    // waveform upload parameters
    long waveform_upload_pause_us; // wait time before a new waveform step is sent to the same FPU
    bool confirm_each_step; // request confirmation for each waveform step

    int min_bus_repeat_delay_ms; // delay, in milliseconds, before writing to the same CAN bus
    int min_fpu_repeat_delay_ms; // delay, in milliseconds, before writing to the same FPU

    int firmware_version_address_offset;
    int can_command_priority; // maximum priority of CAN commands; this is a four-bit value
    int configmotion_max_retry_count; // number of times time-outs
				      // will be reported and missing
				      // data is send again
    int configmotion_max_resend_count; // number of times all data
				       // will be resent silently on a
				       // low level

    EtherCANInterfaceConfig()
        : logLevel(LOG_TRACE_CAN_MESSAGES)
    {
        num_fpus = MAX_NUM_POSITIONERS;

        // set default time-out values

        SocketTimeOutSeconds = 20.0;
        TCP_IdleSeconds = 10;
        TCP_KeepaliveIntervalSeconds = 1;

        waveform_upload_pause_us = 0;
        confirm_each_step = true;
	can_command_priority = 3;
	min_bus_repeat_delay_ms = 2; 
	min_fpu_repeat_delay_ms = 4;
	configmotion_max_retry_count = 10;
	configmotion_max_resend_count = 5;

        firmware_version_address_offset = 0x61; // new offset for v1.3.0, matching firmware version 1.4.4

        // Initialize log file descriptors
        fd_controllog = -1;
        fd_rxlog = -1;
        fd_txlog = -1;

        alpha_datum_offset = ALPHA_DATUM_OFFSET;
        motor_minimum_frequency = 500.0;
        motor_maximum_frequency = 2000.0;
        motor_max_start_frequency=550.0;
        motor_max_rel_increase = 1.4;
    };

};

}
#endif
