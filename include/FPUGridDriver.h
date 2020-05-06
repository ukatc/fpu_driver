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
#include "EtherCANInterface.h"

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
        // NOTE: Boost.Python only allows up to 14 function arguments in
        // function wrappers, so need to keep number of arguments within this
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

    // TODO: Un-comment the virtual destructor below, and figure out how to
    // add it to Boost.Python wrapper - would currently result in the following
    // error:
    // ImportError: ./griddriver.so: undefined symbol: _ZTIN6mpifps21UnprotectedGridDriverE
#if 0
    virtual ~UnprotectedGridDriver();
#endif // 0

    //..........................................................................
    // TODO: Ad-hoc test functions only - remove when no longer needed
    int testIncrement()
    {
        dummyCounter++;

        return dummyCounter;
    }

    double testDivide(double dividend, double divisor)
    {
        return dividend / divisor;
    }

    int testGetNumFPUs()
    {
        return config.num_fpus;
    }

    //..........................................................................

private:
    EtherCANInterfaceConfig config;


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
