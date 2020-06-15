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
// NAME UnprotectedGridDriver.C
//
// TODO: Put description here
//
////////////////////////////////////////////////////////////////////////////////



// ********** NOTE: This file (along with GridDriver.C) is Bart's work in
// progress for converting the classes and functions in FPUGridDriver.py from
// Python to C++.

#include <fcntl.h>
#include <algorithm>
#include "UnprotectedGridDriver.h"
#include "DeviceLock.h"

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

#ifdef FPU_SET_IS_VECTOR

E_EtherCANErrCode UnprotectedGridDriver::check_fpuset(const FpuSelection &fpu_selection)
{
    E_EtherCANErrCode status = DE_OK;

    for (int fpu_id : fpu_selection)
    {
        if ((fpu_id >= config.num_fpus) || (fpu_id >= MAX_NUM_POSITIONERS))
        {
            status = DE_INVALID_FPU_ID;
            break;
        }
    }

    return status;
}

void UnprotectedGridDriver::need_ping(const t_grid_state &grid_state,
                                      const FpuSelection &fpu_selection,
                                      FpuSelection &fpu_ping_selection_ret)
{
    fpu_ping_selection_ret.clear();

    if (fpu_selection.size() == 0)
    {
        // Get all FPUs not yet successfully pinged
        for (int fpu_id = 0;
             fpu_id < std::min<int>(config.num_fpus, MAX_NUM_POSITIONERS);
             fpu_id++)
        {
            if (!grid_state.FPU_state[fpu_id].ping_ok)
            {
                fpu_ping_selection_ret.emplace_back(fpu_id);
            }
        }
    }
    else
    {
        // Get selection of FPUs not yet successfully pinged
        for (int fpu_id : fpu_selection)
        {
            if ((fpu_id < config.num_fpus) &&
                (fpu_id < MAX_NUM_POSITIONERS))   // Buffer overrun protection
            {
                if (!grid_state.FPU_state[fpu_id].ping_ok)
                {
                    fpu_ping_selection_ret.emplace_back(fpu_id);
                }
            }
        }
    }
}

#else // NOTFPU_SET_IS_VECTOR

E_EtherCANErrCode UnprotectedGridDriver::check_fpuset(const AsyncInterface::t_fpuset &fpuset)
{
    E_EtherCANErrCode status = DE_OK;

    // Count number of FPUs selected
    int num_fpus_in_fpuset = 0;
    for (int i = 0; i < MAX_NUM_POSITIONERS; i++)
    {
        if (fpuset[i])
        {
            num_fpus_in_fpuset++;

            // Check if any selected FPU in fpuset has an index above
            // config.num_fpus
            if (i >= config.num_fpus)
            {
                status = DE_INVALID_FPU_ID;
                break;
            }
        }
    }

    if (num_fpus_in_fpuset > config.num_fpus)
    {
        status = DE_INVALID_FPU_ID;
    }

    return status;
}
#endif // NOTFPU_SET_IS_VECTOR

E_EtherCANErrCode UnprotectedGridDriver::findDatum(t_grid_state &gs, 
                    const AsyncInterface::t_datum_search_flags &orig_search_modes,
                    enum E_DATUM_SELECTION selected_arm,
                    const AsyncInterface::t_fpuset &fpuset,
                    bool soft_protection, bool count_protection,
                    bool support_uninitialized_auto,
                    enum E_DATUM_TIMEOUT_FLAG timeout)
{
    E_EtherCANErrCode result = DE_OK;

    /*
    Moves all FPUs to datum position.

    *** TODO: CHECK THESE COMMENTS - copied/adapted from Python version ***

    If the program receives a SIGNINT, or Control-C is pressed, an abortMotion
    command is automatically sent, aborting the search.
    *** TODO: WILL THIS BE THE CASE IN THIS C++ VERSION? ***

    search_modes: Each value is one of SEARCH_CLOCKWISE, SEARCH_ANTI_CLOCKWISE,
    SEARCH_AUTO, SKIP_FPU, which controls whether the datum search moves
    clockwise (decreasing step count), anti-clockwise (increasing step count),
    automatically, or skips the FPU. The default mode is SEARCH_AUTO.

    selected_arm: (DASEL_BOTH, DASEL_ALPHA, DASEL_BETA) controls which arms
    are moved.

    It is critical that the search direction for the beta arm is always set
    correctly, otherwise a beta arm collision will happen which could degrade
    the FPU (or even destroy the FPU, if the collision detection does not
    work). If a beta arm was datumed, the FPU will move in the direction
    corresponding to its internal step count. If an beta arm is not datumed,
    automatic datum search will use the position database value, unless
    support_uninitialized_auto is set to false. In addition, manually-set
    search directions will be checked using the step count value for the beta
    arm, unless count_protection is set to false.

    If a beta arm position appears not to be safe to be moved into the
    requested position, the manual datum search will be refused unless
    soft_protection is set to false.
    */

    result = check_fpuset(fpuset);
    if (result == DE_OK)
    {
        if (soft_protection)
        {
            
        }

    }



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


//******************************************************************************
// Test functions follow
//******************************************************************************

void UnprotectedGridDriver::doTests()
{
    // Ad-hoc tests for single-stepping
 
    //test_check_fpuset();
    
    //test_need_ping();
    
    //test_connect();
            
    test_FindDatum();
}

void UnprotectedGridDriver::test_check_fpuset()
{
    E_EtherCANErrCode status;
    
    int original_num_fpus = config.num_fpus;
    config.num_fpus = 5;
    
#ifdef FPU_SET_IS_VECTOR
    FpuSelection fpu_selection = { 1, 4, 17 };
    status = check_fpuset(fpu_selection); // DE_INVALID_FPU_ID
    fpu_selection = { 1, 2, 3 };
    status = check_fpuset(fpu_selection);   // DE_OK
#endif // NOT FPU_SET_IS_VECTOR
    
    config.num_fpus = original_num_fpus;
}
        
void UnprotectedGridDriver::test_need_ping()
{
    t_grid_state grid_state;
    grid_state.FPU_state[0].ping_ok = true;
    grid_state.FPU_state[1].ping_ok = false;
    grid_state.FPU_state[2].ping_ok = true;
    grid_state.FPU_state[3].ping_ok = true;
    grid_state.FPU_state[4].ping_ok = false;
    grid_state.FPU_state[5].ping_ok = false;
    grid_state.FPU_state[6].ping_ok = false;
    grid_state.FPU_state[7].ping_ok = true;
    grid_state.FPU_state[8].ping_ok = false;
    grid_state.FPU_state[9].ping_ok = true;
    
    int original_num_fpus = config.num_fpus;
    config.num_fpus = 10;

#ifdef FPU_SET_IS_VECTOR
    FpuSelection fpu_ping_selection;
    fpu_selection = { 1, 2, 4, 7 };
    need_ping(grid_state, fpu_selection, fpu_ping_selection);
    fpu_selection = { };
    need_ping(grid_state, fpu_selection, fpu_ping_selection);
    fpu_selection = { 0 };
    need_ping(grid_state, fpu_selection, fpu_ping_selection);
    fpu_selection = { 4 };
    need_ping(grid_state, fpu_selection, fpu_ping_selection);
    fpu_selection = { 6, 7, 8, 9 };
    need_ping(grid_state, fpu_selection, fpu_ping_selection);
#endif // NOT FPU_SET_IS_VECTOR    
    
    config.num_fpus = original_num_fpus;
}

void UnprotectedGridDriver::test_connect()
{
    // TODO: NOTE: I don't yet know what format of IP address is expected -
    // just using a dummy format for now
    // TODO: Also, t_gateway_address::ip is only a pointer - dangerous? Change
    // this eventually? (e.g. to a std::String?)
    const char *dummy_ip_str = "192.168.12.34";
    t_gateway_address gateway_address = { dummy_ip_str, 12345 };
    connect(1, &gateway_address);
}

void UnprotectedGridDriver::test_FindDatum()
{
}


//******************************************************************************
// TODO: Boost.Python wrapper test functions - remove eventually
//******************************************************************************

int UnprotectedGridDriver::boostPythonIncrement()
{
    dummyCounter++;
    return dummyCounter;
}

double UnprotectedGridDriver::boostPythonDivide(double dividend, double divisor)
{
    return dividend / divisor;
}

int UnprotectedGridDriver::boostPythonGetNumFPUs()
{
    return config.num_fpus;
}

//==============================================================================

} // namespace mpifps

