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
    bool confirm_each_step,
    int configmotion_max_retry_count,
    int configmotion_max_resend_count,
    int min_bus_repeat_delay_ms,
    int min_fpu_repeat_delay_ms,
    //enum E_LogLevel logLevel,  // TODO: Figure out how to implement enums in Boost.Python
    const string &log_dir,
    double motor_minimum_frequency,
    double motor_maximum_frequency,
    double motor_max_start_frequency,
    double motor_max_rel_increase
    )
{
    
    // TODO: Fill out this constructor from Python equivalent

}



//==============================================================================

GridDriver::GridDriver()
{
    
}

//==============================================================================


} // namespace mpifps

