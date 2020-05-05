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
// NAME FPUGridDriver.C
//
// TODO: Put description here
//
////////////////////////////////////////////////////////////////////////////////



// ********** NOTE: This file is Bart's work in progress for converting the
// classes and functions in FPUGridDriver.py from Python to C++.


#include "FPUGridDriver.h"

#ifdef DEBUG
#include <stdio.h>
#endif

namespace mpifps
{

//==============================================================================

UnprotectedGridDriver::UnprotectedGridDriver(
    int nfpus,
    double socketTimeOutSeconds,
    bool confirm_each_step,
    int waveform_upload_pause_us,
    int configmotion_max_retry_count,
    int configmotion_max_resend_count,
    int min_bus_repeat_delay_ms,
    int min_fpu_repeat_delay_ms,
    double alpha_datum_offset,
    //enum E_LogLevel logLevel,  // TODO: Figure out how to implement enums in Boost.Python
    const string &log_dir,
    double motor_minimum_frequency,
    double motor_maximum_frequency,
    double motor_max_start_frequency
    // NOTE: Boost.Python only allows up to 14 function params
#if 0
    double motor_max_rel_increase,
    int motor_max_step_difference,
    int firmware_version_address_offset,
    const string &protection_logfile,
    const string &control_logfile,
    const string &tx_logfile,
    const string &rx_logfile,
    const string &start_timestamp
#endif // 0
    )
{
    
    // TODO: Fill out this constructor from Python equivalent

}

// TODO: Ad-hoc test function only - remove when no longer needed
int UnprotectedGridDriver::testFunction()
{
    dummyCounter++;

    return dummyCounter;
}


//==============================================================================

GridDriver::GridDriver()
{
    
}

//==============================================================================


} // namespace mpifps

