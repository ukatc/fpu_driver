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
// NAME E_LogLevel.h
//
// Defines an enumeration which sets the log level in the CAN driver
//
//
////////////////////////////////////////////////////////////////////////////////

#ifndef LOG_LEVEL_H
#define LOG_LEVEL_H

namespace mpifps
{

enum E_LogLevel
{

    // Log only critical errors and important warnings, such as collision
    // messages and message time-outs.
    LOG_ERROR = 0,

    // Also log summary of each command send to the FPU grid, and
    // overall statistics for FPU states (e.g. number of FPUs which
    // have reached the datum position)
    LOG_INFO = 1,

    // Additionally, log the movement targets for each FPU and
    // detailed state of the whole FPU grid on completion of each
    // command.  This level will generate a larger amount of data but
    // will not affect responsiveness of the message processing within
    // the driver. This will be the default log level. It is intended
    // to reconstruct any problem with collisions or hardware defects
    // during normal instrument operation and, when necessary, help to
    // improve collision recovery strategies.
    LOG_GRIDSTATE = 2,

    // Log details of each command sent to each FPU (e.g. sent
    // waveform tables). This level will generate a large amount of
    // data but should usually not affect responsiveness of the
    // driver. This level of logging is appropriate e.g. when
    // debugging the generation of waveform data from the path
    // analysis layer.
    LOG_VERBOSE = 3,

    // Log details on CAN response time-outs and any information which
    // might be helpful to diagnose problems.
    LOG_DEBUG = 4,

    // Additionally, log hex dump of binary data of each CAN message
    // as it is sent to the FPUs and each CAN response. This data will
    // be logged to two additional files. This level will generate a
    // very large amount of data and is appropriate when debugging
    // issues with the CAN message generation, the CAN protocol
    // itself, or issues with the FPU firmware. Because messages are
    // sent from within high-priority event loops, enabling this level
    // will degrade the responsiveness of the driver. It is not
    // designed to be used during normal instrument operation.
    LOG_TRACE_CAN_MESSAGES = 5,

};

#define LOG_CONTROL(minlevel, ...){                                       \
        if ((config.logLevel >= minlevel) && (config.fd_controllog >= 0)) \
        {                                                                 \
            dprintf(config.fd_controllog, __VA_ARGS__);                   \
        }                                                                 \
        }

#define LOG_TX(minlevel, ...){                                            \
        if ((config.logLevel >= minlevel) && (config.fd_txlog >= 0))      \
        {                                                                 \
            dprintf(config.fd_txlog, __VA_ARGS__);                        \
        }                                                                 \
        }

#define LOG_RX(minlevel, ...){                                            \
        if ((config.logLevel >= minlevel) && (config.fd_rxlog >= 0))      \
        {                                                                 \
            dprintf(config.fd_rxlog, __VA_ARGS__);                        \
        }                                                                 \
        }


}

#endif
