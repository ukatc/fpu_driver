// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-04-28  TODO: Put comment here
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME FPUGridDriver.h
//
// TODO: Put description here
//
////////////////////////////////////////////////////////////////////////////////

#ifndef FPUGRIDDRIVER_H
#define FPUGRIDDRIVER_H

#include <string>
#include "E_LogLevel.h"
#include "InterfaceConstants.h"
#include "FPUConstants.h"

using namespace std;

namespace mpifps
{

// *** TODO: Temporary dummy values for now - need to get from the Linux 
// environment variables in same way as is done in FpuGridDriver.py - need to
// figure out the C/C++ equivalents 
#define DEFAULT_NUM_FPUS    (1)
#define DEFAULT_LOGLEVEL    (LOG_ERROR) 
#define DEFAULT_LOGDIR      ("$HOME")


//..............................................................................
    
class UnprotectedGridDriver
{
public:
    UnprotectedGridDriver(
        // NOTE: Boost.Python only allows up to 14 function arguments
        int nfpus = DEFAULT_NUM_FPUS,
        bool confirm_each_step = false,
        int configmotion_max_retry_count = 5,
        int configmotion_max_resend_count = 10,
        int min_bus_repeat_delay_ms = 0,
        int min_fpu_repeat_delay_ms = 1,
        //enum E_LogLevel logLevel = DEFAULT_LOGLEVEL,  // TODO: Figure out how to implement enums in Boost.Python
        const string &log_dir = DEFAULT_LOGDIR,
        double motor_minimum_frequency = MOTOR_MIN_STEP_FREQUENCY,
        double motor_maximum_frequency = MOTOR_MAX_STEP_FREQUENCY,
        double motor_max_start_frequency = MOTOR_MAX_START_FREQUENCY,
        double motor_max_rel_increase = MAX_ACCELERATION_FACTOR
        );

        // TODO: The following are additional arguments defined in
        // FPUGridDriver.py -> UnprotectedGridDriver __init__, but are not
        // used by any higher-level Python scripts so need to be changed to
        // just being internal class defaults
#if 0
        double socketTimeOutSeconds = 20.0,
        int waveform_upload_pause_us = 0,
        double alpha_datum_offset = ALPHA_DATUM_OFFSET,
        int motor_max_step_difference = MAX_STEP_DIFFERENCE,
        int firmware_version_address_offset = 0x61,
        const string &protection_logfile = "_{start_timestamp}-fpu_protection.log",
        const string &control_logfile = "_{start_timestamp}-fpu_control.log",
        const string &tx_logfile = "_{start_timestamp}-fpu_tx.log",
        const string &rx_logfile = "_{start_timestamp}-fpu_rx.log",
        const string &start_timestamp = "ISO8601"
#endif // 0

    // TODO: Un-comment the virtual destructor below, and figure out how to
    // add it to Boost.Python wrapper - would currently result in the following
    // error:
    // ImportError: ./griddriver.so: undefined symbol: _ZTIN6mpifps21UnprotectedGridDriverE
#if 0
    virtual ~UnprotectedGridDriver();
#endif // 0

    // TODO: Ad-hoc test function only - remove when no longer needed
    int testFunction();

private:
    // TODO: Ad-hoc test variable only - remove when no longer needed
    int dummyCounter = 0;

};

class GridDriver
{
public:
    GridDriver();

private:    

};


} // namespace mpifps

#endif // FPUGRIDDRIVER_H
