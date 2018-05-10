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
// NAME GridDriverConfig.h
//
// This file defines a POD structure with pre-set configuration values
//
//
////////////////////////////////////////////////////////////////////////////////

#ifndef GRID_DRIVER_CONFIG_H
#define GRID_DRIVER_CONFIG_H

#include "E_LogLevel.h"
#include "DriverConstants.h"


namespace mpifps
{

struct GridDriverConfig
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

    GridDriverConfig()
    {
        num_fpus = MAX_NUM_POSITIONERS;

        // set default time-out values

        SocketTimeOutSeconds = 20.0;
        TCP_IdleSeconds = 10;
        TCP_KeepaliveIntervalSeconds = 1;

        // Initialize log file descriptors
        logLevel = LOG_TRACE_CAN_MESSAGES;
        fd_controllog = -1;
        fd_rxlog = -1;
        fd_txlog = -1;
    };

};

}
#endif
