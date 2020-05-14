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

#include <fcntl.h>
#include "FPUGridDriver.h"

#ifdef DEBUG
#include <stdio.h>
#endif


namespace mpifps
{
// TODO: Temporary "warn()" placeholder for now - implement something better
#define warn(warnString)

#define TIMESTAMP_INIT_STRING     "ISO8601"

// -----------------------------------------------------------------------------
string get_logname(string basename, string log_dir = "", string timestamp = "")
{

    // TODO: Convert from FpuGridDriver.py -> get_logname()

    if (timestamp == TIMESTAMP_INIT_STRING)
    {

    }

}

// -----------------------------------------------------------------------------
string make_logdir(string log_dir)
{

    // TODO: Convert from FpuGridDriver.py -> make_logdir() - not sure about
    // equivalent C++ path manipulations for Linux path handling yet

    return string("");
}


//==============================================================================

UnprotectedGridDriver::UnprotectedGridDriver(
    int nfpus,
    bool confirm_each_step,
    int configmotion_max_retry_count,
    int configmotion_max_resend_count,
    int min_bus_repeat_delay_ms,
    int min_fpu_repeat_delay_ms,
    enum E_LogLevel logLevel,
    const string &log_dir,
    double motor_minimum_frequency,
    double motor_maximum_frequency,
    double motor_max_start_frequency,
    double motor_max_rel_increase
    )
{

    // TODO: Finish filling out this constructor from Python equivalent



    // self.lock = threading.RLock()   // TODO: Adapt from Python - use e.g. 
                                       // pthread_mutex_lock()? (see EtherCANInterface.C)
                                       // AND need to unlock somehwere else, or would it be
                                       // done automatically due to RAII?

    if (confirm_each_step)
    {
        warn("confirm_each_steps set to True, which requires extra\n"
             "confirmation requests of waveform step upload, and reduces performance");
    }

    if (min_bus_repeat_delay_ms > 0)
    {
        warn("min_bus_repeat_delay_ms is set to value above 0.\n"
             "Decrease if message rate is too low.");
    }

    // Initialise EtherCAN configuration object's parameters
    config.num_fpus = nfpus;
    config.SocketTimeOutSeconds = 20.0;
    config.alpha_datum_offset = ALPHA_DATUM_OFFSET;
    config.motor_minimum_frequency = motor_minimum_frequency;
    config.motor_maximum_frequency = motor_maximum_frequency;
    config.motor_max_start_frequency = motor_max_start_frequency;
    config.motor_max_rel_increase = motor_max_rel_increase;
    config.confirm_each_step = confirm_each_step;
    config.configmotion_max_retry_count = configmotion_max_retry_count;
    config.configmotion_max_resend_count = configmotion_max_resend_count;
    config.waveform_upload_pause_us = 0;
    config.min_bus_repeat_delay_ms = min_bus_repeat_delay_ms;
    config.min_fpu_repeat_delay_ms = min_fpu_repeat_delay_ms;
    config.configmotion_max_resend_count = configmotion_max_resend_count;
    config.configmotion_max_retry_count = configmotion_max_retry_count;
    config.firmware_version_address_offset = 0x61;

    int flags = O_CREAT | O_APPEND | O_WRONLY;
    mode_t mode = 0644;  // Octal

    config.logLevel = logLevel;

    string log_path = make_logdir(log_dir);

    //..........................................................................

    // TODO: Convert log file initialisation code from FpuGridDriver.py

#if 0
    const string &protection_logfile = "_{start_timestamp}-fpu_protection.log",
    const string &control_logfile = "_{start_timestamp}-fpu_control.log",
    const string &tx_logfile = "_{start_timestamp}-fpu_tx.log",
    const string &rx_logfile = "_{start_timestamp}-fpu_rx.log",
    const string &start_timestamp = TIMESTAMP_INIT_STRING;
#endif // 0

    //..........................................................................

    // TODO: How to catch / indicate allocation failure? Add try/catch around
    // the following, OR use std::nothrow?
    _gd = new EtherCANInterface(config);
}

E_EtherCANErrCode UnprotectedGridDriver::connect(const int ngateways,
                                const t_gateway_address gateway_addresses[])
{
    // TODO: Adapt from Python code below

    // TODO: Also implement Python binding for this function - approach can be
    // seen in ethercanif.C -> connectGateways() - binding function can just
    // call it? BUT FPUGridDriver Python version does more with locking etc
    // Adjust the entry parameter format above if necessary


    // N.B. AsyncInterface::connect() / _gd->connect() requires:
    //      (const int ngateways, const t_gateway_address gateway_addresses[])

    E_EtherCANErrCode rv = _gd->connect(ngateways, gateway_addresses);

    return rv;

/*
    with self.lock:
        self.locked_gateways = [] # this del's and releases any previously acquired locks
        for gw in address_list:
            groupname = os.environ.get("MOONS_GATEWAY_ACCESS_GROUP","moons")
            # acquire a unique inter-process lock for each gateway IP
            dl = devicelock.DeviceLock('ethercan-gateway@%s:%i' % (gw.ip, gw.port), groupname)
            self.locked_gateways.append(dl)
        rv = self._gd.connect(address_list)
        self._post_connect_hook(self.config)
        return rv
*/        
}

UnprotectedGridDriver::~UnprotectedGridDriver()
{
    if (_gd != nullptr)
    {
        delete _gd;
    }

    // TODO: Close/delete any other non-RAII objects here - see 
    // FpuGridDriver.py -> UnprotectedGridDriver -> __del__

}


//==============================================================================

GridDriver::GridDriver()
{
    
}

//==============================================================================


} // namespace mpifps

