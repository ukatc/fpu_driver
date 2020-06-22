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
#include <unistd.h>
#include "UnprotectedGridDriver.h"
#include "DeviceLock.h"
#include "FPUArray.h"

#ifdef DEBUG
#include <stdio.h>
#endif


namespace mpifps
{
// TODO: Temporary "warn()" placeholder for now - implement something better
#define warn(warnString)

#define TIMESTAMP_INIT_STRING     "ISO8601"

//------------------------------------------------------------------------------
string get_logname(string basename, string log_dir = "", string timestamp = "")
{

    // TODO: Convert from FpuGridDriver.py -> get_logname()

    if (timestamp == TIMESTAMP_INIT_STRING)
    {

    }

}

//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
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


//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
void UnprotectedGridDriver::need_ping(const t_grid_state &gs,
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
            if (!gs.FPU_state[fpu_id].ping_ok)
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
                if (!gs.FPU_state[fpu_id].ping_ok)
                {
                    fpu_ping_selection_ret.emplace_back(fpu_id);
                }
            }
        }
    }
}

#else // NOTFPU_SET_IS_VECTOR

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::check_fpuset(const t_fpuset &fpuset)
{
    E_EtherCANErrCode status = DE_OK;

    // Count number of FPUs selected
    int num_fpus_in_fpuset = 0;
    for (int fpu_id = 0; fpu_id < MAX_NUM_POSITIONERS; fpu_id++)
    {
        if (fpuset[fpu_id])
        {
            num_fpus_in_fpuset++;

            // Check if any selected FPU in fpuset has an index above
            // config.num_fpus
            if (fpu_id >= config.num_fpus)
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

//------------------------------------------------------------------------------
void UnprotectedGridDriver::need_ping(const t_grid_state &gs,
                                      const t_fpuset &fpuset,
                                      t_fpuset &pingset_ret)
{
    
    // TODO: Implement this function
    
}

#endif // NOTFPU_SET_IS_VECTOR

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::findDatum(t_grid_state &gs, 
                    const t_datum_search_flags &orig_search_modes,
                    enum E_DATUM_SELECTION selected_arm, const t_fpuset &fpuset,
                    bool soft_protection, bool count_protection,
                    bool support_uninitialized_auto,
                    enum E_DATUM_TIMEOUT_FLAG timeout)
{
    E_EtherCANErrCode result = DE_ERROR_UNKNOWN;
    E_EtherCANErrCode result_returned = DE_ERROR_UNKNOWN;

    /*
    Moves all FPUs to datum position.

    *** TODO: This function conversion from Python version is still WIP ***

    TODO: Eventually rename orig_search_modes above to just search_modes (and
    rename search_modes inside this function to e.g. search_modes_adjusted) -
    but the current names are being used for now to match the Python version

    *** TODO: CHECK THE COMMENTS BELOW - copied/adapted from Python version ***

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
        // TODO: Add C++/Linux equivalent of Python version's "with self.lock"
        // here 

        // Make temporary local copy of search modes to allow local modification
        t_datum_search_flags search_modes;
        for (int i = 0; i < MAX_NUM_POSITIONERS; i++)
        {
            search_modes[i] = orig_search_modes[i];
        }

        if (soft_protection)
        {
            // Check whether datum search is safe, adjusting search_modes
            _allow_find_datum_hook(gs, search_modes, selected_arm,
                                   fpuset, support_uninitialized_auto);
            // We might need to disable the stepcount-based check if (and only
            // if) the step counter does not agree with the database.
            // Check whether a movement against the step count is needed.
            for (int fpu_id = 0; fpu_id < MAX_NUM_POSITIONERS; fpu_id++)
            {
                if ( ((search_modes[fpu_id] == SEARCH_CLOCKWISE) ||
                      (search_modes[fpu_id] == SEARCH_ANTI_CLOCKWISE)) &&
                     (orig_search_modes[fpu_id] == SEARCH_AUTO))
                {
                    // TODO: Check against equivalent Python, that
                    // count_protection is only required to be modified
                    // locally inside this function, and does not alter
                    // the function argument value
                    count_protection = false;
                    break;
                }
            }
        }
        
        FpuPositions initial_positions;
        _start_find_datum_hook(gs, search_modes, selected_arm, fpuset,
                               initial_positions, soft_protection);

        t_grid_state prev_gs;
        _gd->getGridState(prev_gs);

        result = _gd->startFindDatum(gs, search_modes, selected_arm, timeout,
                                     count_protection, &fpuset);
        if (result != DE_OK)
        {
            // We cancel the datum search altogether, so we can reset
            // positions to old value
            _cancel_find_datum_hook(gs, fpuset, initial_positions);

            // TODO: Is this a good way to handle the error? (the Python
            // version does a "raise")
            return result;
        }

        double time_interval = 0.1;
        usleep((useconds_t)(time_interval * 1000000.0));
        bool is_ready = false;
        bool was_aborted = false;
        bool finished_ok = false;

        // TODO: In the Python equivalent code, has code to allow aborting
        // from a signal (see was_aborted etc) - but this isn't appropriate
        // in this C++ version because will be part of ESO driver? Come back
        // to this / check what kind of ESO driver abort mechanism is required
        //
        // TODO: In the Python equivalent code, has try/finally block - but
        // is this required/useful/appropriate in this C++ code - are there 
        // any exceptions to be caught?
        while (!is_ready)
        {
            bool wait_find_datum_finished = false;
            result = _gd->waitFindDatum(gs, time_interval,
                                        wait_find_datum_finished, &fpuset);

            // TODO: The following is extra from
            // WrapEtherCANInterface::wrap_waitFindDatum() - check that it's
            // OK
            if (((!wait_find_datum_finished) && (result == DE_OK))
                    || (result == DE_WAIT_TIMEOUT))
            {
                result = DE_WAIT_TIMEOUT;
            }
            is_ready = (result != DE_WAIT_TIMEOUT);
            finished_ok = (result == DE_OK);
        }

        if (!finished_ok)
        {
            usleep((useconds_t)(time_interval * 1000000.0));

            t_fpuset pingset;
            need_ping(gs, fpuset, pingset);

            // Perform FPU pinging if any FPU needs to be pinged
            bool need_pinging = false;
            for (int fpu_id = 0; fpu_id < MAX_NUM_POSITIONERS; fpu_id++)
            {
                if (pingset[fpu_id])
                {
                    need_pinging = true;
                    break;
                }
            }
            if (need_pinging)
            {
                result = _pingFPUs(gs, pingset);
                if (result != DE_OK)
                {
                    // TODO: Error
                }
            }
        }

        _finished_find_datum_hook(prev_gs, gs, search_modes, fpuset,
                                  !finished_ok, initial_positions,
                                  selected_arm);

        if (was_aborted)
        {

            // TODO: Not appropriate to abort etc? (see comments further up in
            // this function)

        }
    }


    // TODO: Generate good return codes in the code above

    return result_returned;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::_pingFPUs(t_grid_state &gs,
                                                   const t_fpuset &fpuset)
{
    E_EtherCANErrCode result = DE_ERROR_UNKNOWN;

    result = check_fpuset(fpuset);
    if (result == DE_OK)
    {
        result = _gd->pingFPUs(gs, fpuset);
    }

    return result;
}

//------------------------------------------------------------------------------
bool UnprotectedGridDriver::wavetable_was_received(const t_wtable &wtable,
                                                   int fpu_id,
                                                   const t_fpu_state &fpu_state,  
                                                   bool allow_unconfirmed,
                                                   E_FPU_STATE target_state)
{
    bool wtable_contains_fpu_id = false;
    if ((fpu_id >= 0) && (fpu_id < MAX_NUM_POSITIONERS))
    {
        for (const auto &it : wtable)
        {
            if (it.fpu_id == fpu_id)
            {
                wtable_contains_fpu_id = true;
                break;
            }
        }
    }

    return (wtable_contains_fpu_id && (fpu_state.state == target_state) &&
            ((fpu_state.last_status == 0) || (allow_unconfirmed &&
                    (fpu_state.last_status == MCE_NO_CONFIRMATION_EXPECTED))));
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::configMotion(const t_wtable &wavetable,
                                t_grid_state &gs, const t_fpuset &fpuset,
                                bool soft_protection, bool allow_uninitialized,
                                int ruleset_version, bool warn_unsafe,
                                int verbosity)
{
    /*
    Configures movement by sending a waveform table to a group of FPUs.

    When the 'protected' flag is set to False, bypass all hardware protection
    checks, which will allow to move a collided or uncalibrated FPU (even if
    the movement might damage the hardware).
    */

    // TODO: NOTE: UnprotectedGridDriver::_post_config_motion_hook() DOES also
    // have a body (see UnprotectedGridDriver.h), in addition to
    // GridDriver::_post_config_motion_hook() - check it

    // TODO: Sort out return values

    E_EtherCANErrCode result = DE_ERROR_UNKNOWN;
    result = check_fpuset(fpuset);
    if (result == DE_OK)
    {
        // TODO: Add C++/Linux equivalent of Python version's "with self.lock"
        // here 

        // We make a copy of the wavetable to make sure no side effects are
        // visible to the caller
        // TODO: Will the following statement do a deep copy as required?
        // TODO: This will use lots of local stack - is the stack size big
        // enough for this?
        t_wtable wtable = wavetable;

        // Delete all wtable elements not selected in fpuset
        // TODO: This might be very inefficient because deleting std::vector
        // elements from middle - but will work OK
        // TODO: Test this carefully
        for (auto it = wtable.begin(); it != wtable.end(); )
        {
            bool erased = false;
            if (it->fpu_id < MAX_NUM_POSITIONERS)   // Sanity check
            {
                if (!fpuset[it->fpu_id])
                {
                    // N.B. erase() returns a new (valid) iterator that points
                    // to the next element after the deleted one, so it's a
                    // safe operation
                    it = wtable.erase(it);
                    erased = true;
                }
            }
            else
            {
                // TODO: Error - flag this somehow?
                break;
            }

            if (!erased)
            {
                it++;
            }
        }

        // Check whether wavetable is safe, and if so, register it
        Range wmode;
        if (soft_protection)
        {
            wmode = Range::Error;
        }
        else
        {
            if (warn_unsafe)
            {
                wmode = Range::Warn;
            }
            else
            {
                // TODO: Is this correct?
                wmode = Range::Error;
            }
        }

        _pre_config_motion_hook(wtable, gs, fpuset, wmode);
        bool update_config = false;
        t_grid_state prev_gs;
        _gd->getGridState(prev_gs);

        // TODO: Carefulyl check the following logic against the 3-deep-nested
        // "try" blocks in the equivalent Python code
        result = _gd->configMotion(wtable, gs, fpuset, allow_uninitialized,
                                   ruleset_version);
        // TODO: Check for the various expected result values - some non-DE_OK
        // ones might actually be OK?
        if (result != DE_OK)
        {
            update_config = true;

            // TODO: Catch the more serious errors like the Python code does:
            // InvalidWaveformException, InvalidStateException, SocketFailure,
            // CommandTimeout etc
            // e.g. If SocketFailure or CommandTimeout then would be a
            // transmission failure. Here, it is possible that some FPUs have
            // finished loading valid data, but the process was not finished
            // for all FPUs.

        }

        if (update_config)
        {
            // Accept configured wavetable entries
            for (int fpu_id = 0; fpu_id < MAX_NUM_POSITIONERS; fpu_id++)
            {
                if (wavetable_was_received(wtable, fpu_id, gs.FPU_state[fpu_id]))
                {
                    // ******** TODO: The following statement doesn't build yet
                    // because type mismatch

                    last_wavetable[fpu_id] = wtable[fpu_id];
                }
            }


            _post_config_motion_hook(wtable, gs, fpuset);
        }





    }


}

//------------------------------------------------------------------------------
void UnprotectedGridDriver::set_wtable_reversed(const t_fpuset &fpuset,
                                                bool is_reversed)
{
    // TODO: Add C++/Linux equivalent of Python version's "with self.lock"
    // here 

    for (int fpu_id = 0; fpu_id < MAX_NUM_POSITIONERS; fpu_id++)
    {
        if (fpuset[fpu_id])
        {
            // TODO: wf_reversed is currently a std::map, but should probably
            // be a fixed-size array instead - see comments above its definition
            wf_reversed[fpu_id] = is_reversed;
        }
    }
}

//------------------------------------------------------------------------------
UnprotectedGridDriver::~UnprotectedGridDriver()
{
    if (_gd != nullptr)
    {
        delete _gd;
    }

    // TODO: Close/delete any other non-RAII objects here - see 
    // FpuGridDriver.py -> UnprotectedGridDriver -> __del__

}

//..............................................................................
// TODO: Boost.Python wrapper test member functions - remove eventually
//..............................................................................

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

void UnprotectedGridDriverTester::doTests()
{
    //test_check_fpuset();
    
    //test_need_ping();
    
    //test_connect();
    
    test_findDatum();
}

void UnprotectedGridDriverTester::test_check_fpuset()
{
    E_EtherCANErrCode status;
    
#ifdef FPU_SET_IS_VECTOR
    FpuSelection fpu_selection = { 1, 4, 17 };
    status = check_fpuset(fpu_selection); // DE_INVALID_FPU_ID
    fpu_selection = { 1, 2, 3 };
    status = check_fpuset(fpu_selection);   // DE_OK
#endif // NOT FPU_SET_IS_VECTOR
}

void UnprotectedGridDriverTester::test_need_ping()
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
}

void UnprotectedGridDriverTester::test_connect()
{
    // TODO: NOTE: I don't yet know what format of IP address is expected -
    // just using a dummy format for now
    // TODO: Also, t_gateway_address::ip is only a pointer - dangerous? Change
    // this eventually? (e.g. to a std::String?)
    const char *dummy_ip_str = "192.168.12.34";
    t_gateway_address gateway_address = { dummy_ip_str, 12345 };
    
    UnprotectedGridDriver ugd;
    ugd.connect(1, &gateway_address);
    
}

void UnprotectedGridDriverTester::test_findDatum()
{
    UnprotectedGridDriver ugd;
    E_EtherCANErrCode result;

    // Create and initialise grid_state
    EtherCANInterfaceConfig iface_config;
    FPUArray fpu_array(iface_config);
    t_grid_state grid_state;
    fpu_array.getGridState(grid_state);
    
    // TODO: Initialise these data structures/arrays
    t_datum_search_flags search_modes;
    t_fpuset fpuset;
    
    bool soft_protection = true;
    bool count_protection = true;
    bool support_uninitialized_auto = false;
    
    result = ugd.findDatum(grid_state, search_modes, DASEL_BOTH, fpuset,
                           soft_protection, count_protection,
                           support_uninitialized_auto, DATUM_TIMEOUT_DISABLE);

}


//==============================================================================


} // namespace mpifps

