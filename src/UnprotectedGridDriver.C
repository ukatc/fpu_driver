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

//------------------------------------------------------------------------------
string get_logname(string basename, string log_dir = "", string timestamp = "")
{

    // TODO: Convert from FpuGridDriver.py -> get_logname()

    if (timestamp == DEFAULT_START_TIMESTAMP)
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
    // NOTE: Boost.Python only allows up to 14 function arguments in
    // function wrappers, so need to keep number of arguments within this
    int nfpus,
    double SocketTimeOutSeconds,
    bool confirm_each_step,
    long waveform_upload_pause_us,
    int configmotion_max_retry_count,
    int configmotion_max_resend_count,
    int min_bus_repeat_delay_ms,
    int min_fpu_repeat_delay_ms,
    double alpha_datum_offset,
    double motor_minimum_frequency,
    double motor_maximum_frequency,
    double motor_max_start_frequency,
    double motor_max_rel_increase,
    double motor_max_step_difference)
{
    config.num_fpus = nfpus;
    config.SocketTimeOutSeconds = SocketTimeOutSeconds;
    config.confirm_each_step = confirm_each_step;
    config.waveform_upload_pause_us = waveform_upload_pause_us;
    config.configmotion_max_retry_count = configmotion_max_retry_count;
    config.configmotion_max_resend_count = configmotion_max_resend_count;
    config.min_bus_repeat_delay_ms = min_bus_repeat_delay_ms;
    config.min_fpu_repeat_delay_ms = min_fpu_repeat_delay_ms;
    config.alpha_datum_offset = alpha_datum_offset;
    config.motor_minimum_frequency = motor_minimum_frequency;
    config.motor_maximum_frequency = motor_maximum_frequency;
    config.motor_max_start_frequency = motor_max_start_frequency;
    config.motor_max_rel_increase = motor_max_rel_increase;
    config.motor_max_step_difference = motor_max_step_difference;

    for (int fpu_id = 0; fpu_id < MAX_NUM_POSITIONERS; fpu_id++)
    {
        wf_reversed[fpu_id] = false;
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

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::initialize(
                                E_LogLevel logLevel,
                                const std::string &log_dir,
                                int firmware_version_address_offset,
                                const std::string &protection_logfile,
                                const std::string &control_logfile,
                                const std::string &tx_logfile,
                                const std::string &rx_logfile,
                                const std::string &start_timestamp)
{
    // This function performs further initialisations. It is required to
    // be separate from the constructor for supporting Boost.Python wrapping,
    // because Boost.Python only supports up to 14 function arguments, so
    // can't supply all of the 20-plus required initialisation arguments via
    // the constructor alone.

    config.logLevel = logLevel;
    config.firmware_version_address_offset = firmware_version_address_offset;

    // TODO: If required, this function can fail and return an error code, so
    // that the calling code (which could be the Python wrapper, or eventual
    // ESO driver code) can produce an appropriate error indication?

    // TODO: Finish filling out this constructor from Python equivalent

    // self.lock = threading.RLock()   // TODO: Adapt from Python - use e.g. 
                                       // pthread_mutex_lock()? (see EtherCANInterface.C)
                                       // AND need to unlock somehwere else, or would it be
                                       // done automatically due to RAII?


    //..........................................................................

    // TODO: Convert log file initialisation code from FpuGridDriver.py

    int flags = O_CREAT | O_APPEND | O_WRONLY;
    mode_t mode = 0644;  // Octal

    std::string log_path = make_logdir(log_dir);


    //..........................................................................

    // TODO: How to catch / indicate allocation failure? Add try/catch around
    // the following, OR use std::nothrow?

    E_EtherCANErrCode result;
    _gd = new EtherCANInterface(config);

    if (_gd != nullptr)
    {
        initialize_was_called_ok = true;

        // TODO: Is this the correct place to call initializeInterface()?
        // (Johannes' Python wrapper code calls it from the WrapEtherCANInterface
        // constructor)
        // TODO: The Python wrapper code also has a checkInterfaceError(ecode) call
        // after it (to flag any initialisation error to Python) - how to allow
        // mimicking of this in WrappedGridDriver?
        // TODO: Call deInitialize() from destructor, or elsewhere? BUT is already
        // done from AsyncInterface destructor?
        result = _gd->initializeInterface();
    }
    else
    {
        // TODO: Fatal error - is this the right return code?
        result = DE_OUT_OF_MEMORY;
    }

    return result;
}

//------------------------------------------------------------------------------
bool UnprotectedGridDriver::initializeWasCalledOk()
{
    return initialize_was_called_ok;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::connect(int ngateways,
                                const t_gateway_address gateway_addresses[])
{
    if (!initialize_was_called_ok)
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    // TODO: Also add gateway locking/unlocking code like the Python version
    // of this function does? (see below). If so then do NOT use my
    // DeviceLock.C/h WIP conversion from the Python-equivalent module
    // because too clunky - instead, use Linux named semaphores?

    E_EtherCANErrCode result = _gd->connect(ngateways, gateway_addresses);
    _post_connect_hook(config);
    return result;

/*
    Original Python version of function:
    
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

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::disconnect()
{
    if (!initialize_was_called_ok)
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    return _gd->disconnect();
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::check_fpuset(const t_fpuset &fpuset)
{
    // Check that config.num_fpus is within range
    if ((config.num_fpus < 0) || (config.num_fpus >= MAX_NUM_POSITIONERS))
    {
        // TODO: Any better error return code for this?
        return DE_INVALID_FPU_ID;
    }

    // Check if any selected FPU in fpuset has index >= (config.num_fpus - 1)
    for (int fpu_id = config.num_fpus; fpu_id < MAX_NUM_POSITIONERS; fpu_id++)
    {
        if (fpuset[fpu_id])
        {
            return DE_INVALID_FPU_ID;
        }
    }

    return DE_OK;
}

//------------------------------------------------------------------------------
void UnprotectedGridDriver::need_ping(const t_grid_state &gs,
                                      const t_fpuset &fpuset,
                                      t_fpuset &pingset_ret)
{
    for (int i = 0; i < MAX_NUM_POSITIONERS; i++)
    {
        pingset_ret[i] = false;
    }
    
    for (int i = 0; (i < config.num_fpus) && (i < MAX_NUM_POSITIONERS); i++)
    {
        if (fpuset[i] && (!gs.FPU_state[i].ping_ok))
        {
            pingset_ret[i] = true;
        }
    }    
}

//------------------------------------------------------------------------------
E_GridState UnprotectedGridDriver::getGridState(t_grid_state &grid_state_ret)
{
    if (!initialize_was_called_ok)
    {
        return GS_UNKNOWN;
    }

    return _gd->getGridState(grid_state_ret);
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::findDatum(t_grid_state &gs, 
                    const t_datum_search_flags &orig_search_modes,
                    enum E_DATUM_SELECTION selected_arm, const t_fpuset &fpuset,
                    bool soft_protection, bool count_protection,
                    bool support_uninitialized_auto,
                    enum E_DATUM_TIMEOUT_FLAG timeout)
{
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
    corresponding to its internal step count. If a beta arm is not datumed,
    automatic datum search will use the position database value, unless
    support_uninitialized_auto is set to false. In addition, manually-set
    search directions will be checked using the step count value for the beta
    arm, unless count_protection is set to false.

    If a beta arm position appears not to be safe to be moved into the
    requested position, the manual datum search will be refused unless
    soft_protection is set to false.
    */

    if (!initialize_was_called_ok)
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    E_EtherCANErrCode result = check_fpuset(fpuset);
    if (result != DE_OK)
    {
        return result;
    }

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
        for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
        {
            if (fpuset[fpu_id] && 
                (((search_modes[fpu_id] == SEARCH_CLOCKWISE) ||
                  (search_modes[fpu_id] == SEARCH_ANTI_CLOCKWISE)) &&
                 (orig_search_modes[fpu_id] == SEARCH_AUTO)) )
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

    t_fpu_positions initial_positions;
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

        return result;
    }

    double time_interval = 0.1;
    usleep((useconds_t)(time_interval * microsecs_in_1_sec));
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
        is_ready = (result != DE_WAIT_TIMEOUT);
        finished_ok = (result == DE_OK);
    }

    if (!finished_ok)
    {
        usleep((useconds_t)(time_interval * microsecs_in_1_sec));

        E_EtherCANErrCode ping_if_needed_result = pingIfNeeded(gs, fpuset);
        if (ping_if_needed_result != DE_OK)
        {
            return ping_if_needed_result;
        }
    }

    _finished_find_datum_hook(prev_gs, gs, search_modes, fpuset,
                              !finished_ok, initial_positions,
                              selected_arm);

    if (was_aborted)
    {

        // TODO: See Python equivalent - not appropriate to abort etc? (also
        // see comments further up in this function)

    }

    return result;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::pingIfNeeded(t_grid_state &gs,
                                                      const t_fpuset &fpuset)
{
    // Perform FPU pinging if any FPU needs to be pinged

    t_fpuset pingset;
    need_ping(gs, fpuset, pingset);

    bool any_to_ping = false;
    for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
    {
        if (fpuset[fpu_id] && pingset[fpu_id])
        {
            any_to_ping = true;
            break;
        }
    }
    if (any_to_ping)
    {
        return _pingFPUs(gs, pingset);
    }
    return DE_OK;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::_pingFPUs(t_grid_state &gs,
                                                   const t_fpuset &fpuset)
{
    if (!initialize_was_called_ok)
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    E_EtherCANErrCode result = check_fpuset(fpuset);
    if (result == DE_OK)
    {
        result = _gd->pingFPUs(gs, fpuset);
    }

    return result;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::pingFPUs(t_grid_state &gs,
                                                  const t_fpuset &fpuset)
{
    // Communicates with all FPUs and queries their status
    return _pingFPUs(gs, fpuset);
}

//------------------------------------------------------------------------------
void UnprotectedGridDriver::_post_config_motion_hook(const t_wtable &wtable,
                                                     t_grid_state &gs,
                                                     const t_fpuset &fpuset)
{
    // TODO: Add C++/Linux equivalent of Python version's "with self.lock"
    // here

    set_wtable_reversed(fpuset, false);
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
            ((fpu_state.last_status == MCE_FPU_OK) || (allow_unconfirmed &&
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

    if (!initialize_was_called_ok)
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    // TODO: NOTE: UnprotectedGridDriver::_post_config_motion_hook() DOES also
    // have a body (see UnprotectedGridDriver.h), in addition to
    // GridDriver::_post_config_motion_hook() - check it

    // TODO: Sort out return values

    E_EtherCANErrCode result = check_fpuset(fpuset);
    if (result != DE_OK)
    {
        return result;
    }

    // TODO: Add C++/Linux equivalent of Python version's "with self.lock"
    // here 

    // We make a copy of the wavetable to make sure no side effects are
    // visible to the caller
    // TODO: This will use lots of local stack - is the stack size big
    // enough for this?
    t_wtable wtable = wavetable;

    // Delete all wtable elements not specified in fpuset
    // TODO: This might be very inefficient because deleting std::vector
    // elements from middle - but will work OK
    // TODO: Test this carefully
    for (auto it = wtable.begin(); it != wtable.end(); )
    {
        if ((it->fpu_id >= 0) &&
            (it->fpu_id < config.num_fpus) &&
            (it->fpu_id < MAX_NUM_POSITIONERS))   // Sanity checks
        {
            if (!fpuset[it->fpu_id])
            {
                // N.B. erase() returns a new (valid) iterator that points
                // to the next element after the deleted one, so it's a
                // safe operation
                it = wtable.erase(it);
            }
            else
            {
                it++;
            }
        }
        else
        {
            // Error: Bad FPU ID found in wavetable
            return DE_INVALID_FPU_ID;
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
            wmode = Range::Ignore;
        }
    }

    _pre_config_motion_hook(wtable, gs, fpuset, wmode);
    bool update_config = false;
    // TODO: The following is a quite large data structure which will
    // be stored on the local stack - is this OK? (stack overflow?)
    // Or, better to create it on the heap using a std::unique_ptr?
    // (N.B. But it's an array, so if use unique_ptr then would need to
    // encapsulate the array into a structure or class)
    t_grid_state prev_gs;
    _gd->getGridState(prev_gs);

    // TODO: The equivalent Python code here has 3-deep-nested "try" blocks,
    // and it is not yet clear what the equivalent C++ code needs to be here.
    //
    // TODO: configMotion(), and configMotionAsync() which it calls, can
    // return a large number of different error codes - see what needs
    // to be done here relative to the Python implementation, and also
    // what the equivalent C++ response for the Python version's 
    // InvalidWaveformException, InvalidStateException, SocketFailure,
    // CommandTimeout etc would be (if required)
    //
    // e.g. If SocketFailure or CommandTimeout then would be a
    // transmission failure. Here, it is possible that some FPUs have
    // finished loading valid data, but the process was not finished
    // for all FPUs.

    result = _gd->configMotion(wtable, gs, fpuset, allow_uninitialized,
                               ruleset_version);
    // TODO: Check for the various expected result values - might some
    // non-DE_OK ones might actually be OK?
    if (result != DE_OK)
    {
        update_config = true;
    }

    if (update_config)
    {
        // TODO: Much of the following code is inefficient because iterates
        // through arrays/lists many times - but should work OK. This approach
        // is currently taken in order to conform with the existing C++ data
        // structures already defined.

        // Accept configured wavetable entries
        for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
        {
            // Check if wtable has an fpu_id entry
            bool wtable_contains_fpu_id = false;
            auto it = wtable.begin();
            for ( ; it != wtable.end(); it++)
            {
                if (it->fpu_id == fpu_id)
                {
                    wtable_contains_fpu_id = true;
                    break;
                }
            }
            if (!wtable_contains_fpu_id)
            {
                continue;
            }

            if (wavetable_was_received(wtable, fpu_id, gs.FPU_state[fpu_id]))
            {
                // Add to last_wavetable list (or overwrite existing entry)
                bool fpu_id_found = false;
                for (size_t i = 0; i < last_wavetable.size(); i++)
                {
                    if (last_wavetable[i].fpu_id == fpu_id)
                    {
                        last_wavetable[i] = *it;
                        fpu_id_found = true;
                        break;
                    }
                }
                if (!fpu_id_found)
                {
                    last_wavetable.push_back(*it);
                }
            }
            else
            {
                wtable.erase(it);
            }
            
            _update_error_counters(prev_gs.FPU_state[fpu_id],
                                   gs.FPU_state[fpu_id]);
        }

        _post_config_motion_hook(wtable, gs, fpuset);
    }

    // TODO: Is this return variable correct? (does correspond to equivalent 
    // Python code)
    return result;
}

//------------------------------------------------------------------------------
void UnprotectedGridDriver::set_wtable_reversed(const t_fpuset &fpuset,
                                                bool is_reversed)
{
    // TODO: Add C++/Linux equivalent of Python version's "with self.lock"
    // here 

    for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
    {
        if (fpuset[fpu_id])
        {
            wf_reversed[fpu_id] = is_reversed;
        }
    }
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::executeMotion(t_grid_state &gs,
                                                       const t_fpuset &fpuset,
                                                       bool sync_command)
{
    if (!initialize_was_called_ok)
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    E_EtherCANErrCode result = check_fpuset(fpuset);
    if (result != DE_OK)
    {
        return result;
    }

    // TODO: Add C++/Linux equivalent of Python version's "with self.lock"
    // here

    // Wait a short moment to avoid spurious collision
    t_fpu_positions initial_positions;
    _start_execute_motion_hook(gs, fpuset, initial_positions);
    usleep((useconds_t)(0.1 * microsecs_in_1_sec));
    t_grid_state prev_gs;
    _gd->getGridState(prev_gs);
    usleep((useconds_t)(0.1 * microsecs_in_1_sec));
    result = _gd->startExecuteMotion(gs, fpuset, sync_command);
    // TODO: The result logic here might not be the same as the Python
    // equivalent - check this
    if (result != DE_OK)
    {
        _cancel_execute_motion_hook(gs, fpuset, initial_positions);
        return result;
    }

    double time_interval = 0.1;
    bool is_ready = false;
    bool was_aborted = false;
    bool refresh_state = false;
    // TODO: The result logic here might not be the same as the Python
    // equivalent - check this
    while (!is_ready)
    {
        bool wait_execute_motion_finished = false;
        result = _gd->waitExecuteMotion(gs, time_interval,
                                        wait_execute_motion_finished,
                                        fpuset);
        
        // TODO: Python version can be interrupted from Linux signals here,
        // but this isn't appropriate for C++ version because it wlll be
        // inside ESO driver?

        // (The following check was brought in from 
        // ethercanif.C -> WrapEtherCANInterface::wrap_waitExecuteMotion())
        if (!wait_execute_motion_finished && (result == DE_OK))
        {
            result = DE_WAIT_TIMEOUT;
        }

        is_ready = (result != DE_WAIT_TIMEOUT);
    }

    if (result != DE_OK)
    {
        refresh_state = true;
    }

    // This is skipped in case of a SocketFailure, for example
    if ((result == DE_OK) || refresh_state)
    {
        // Execute a ping to update positions
        // (this is only needed for protocol version 1)  <== TODO: Check this
        t_grid_state move_gs;
        _gd->getGridState(move_gs);

        usleep((useconds_t)(0.1 * microsecs_in_1_sec));

        E_EtherCANErrCode ping_if_needed_result = pingIfNeeded(gs, fpuset);
        if (ping_if_needed_result != DE_OK)
        {
            return ping_if_needed_result;
        }

        // The following hook will narrow down the recorded intervals of
        // positions
        _post_execute_motion_hook(gs, prev_gs, move_gs, fpuset);
    }

    // TODO: Original Python code - need to do anything with this here?
    //if was_aborted:
    //raise MovementError("executeMotion was aborted by SIGINT")

    return result;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::enableMove(int fpu_id, t_grid_state &gs)
{
    // TODO: Add C++/Linux equivalent of Python version's "with self.lock"
    // here

    if (!initialize_was_called_ok)
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    t_grid_state prev_gs;
    _gd->getGridState(prev_gs);

    E_EtherCANErrCode result = _gd->enableMove(fpu_id, gs);

    // TODO: The following line was in original Python FpuGridDriver.enableMove(),
    // but not needed here?
    // status = gs.FPU[fpu_id].last_status

    for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
    {
        _update_error_counters(prev_gs.FPU_state[fpu_id],
                               gs.FPU_state[fpu_id]);
    }

    return result;
}


//==============================================================================
void UnprotectedGridDriverTester::doTests()
{
    E_EtherCANErrCode result;
    E_GridState grid_state_result;
    t_grid_state grid_state;
    const int num_fpus = 1;
    const bool soft_protection = false;
    const double microsecs_in_1_sec = 1000000.0;

    //..........................................................................

    UnprotectedGridDriver ugd(num_fpus);

    //..........................................................................
    // Test initialize()
    ugd.initialize();    

    //..........................................................................
    // Test connect()
    // TODO: t_gateway_address::ip is only a pointer - dangerous? Change this
    // eventually? (e.g. to a std::string?)
    const char *ip_address_str = "127.0.0.1";
    uint16_t port_number = 4700;
    t_gateway_address gateway_address = { ip_address_str, port_number };
    result = ugd.connect(1, &gateway_address);   

    //..........................................................................
    // Specify FPUs in fpuset
    t_fpuset fpuset;
    for (int fpu_id = 0; fpu_id < MAX_NUM_POSITIONERS; fpu_id++)
    {
        fpuset[fpu_id] = false;
    }
    for (int fpu_id = 0; fpu_id < num_fpus; fpu_id++)
    {
        fpuset[fpu_id] = true;
    }

    //..........................................................................
    // Test getGridState()
    grid_state_result = ugd.getGridState(grid_state);

    //..........................................................................
    // Test pingFPUs()
    result = ugd.pingFPUs(grid_state, fpuset);
    
    //..........................................................................
    // Test getGridState() again
    grid_state_result = ugd.getGridState(grid_state);

    //..........................................................................
    // Test pingFPUs() again
    result = ugd.pingFPUs(grid_state, fpuset);
    
    //..........................................................................
    // Test findDatum()
    t_datum_search_flags search_modes;
    for (int fpu_id = 0; fpu_id < num_fpus; fpu_id++)
    {
        search_modes[fpu_id] = SEARCH_CLOCKWISE;
    }
    const bool count_protection = false;
    const bool support_uninitialized_auto = false;

    ugd.getGridState(grid_state);

    result = ugd.findDatum(grid_state, search_modes, DASEL_BOTH, fpuset,
                           soft_protection, count_protection,
                           support_uninitialized_auto, DATUM_TIMEOUT_ENABLE);

    //..........................................................................
    // Test configMotion()
    const bool allow_uninitialized = true;
    const int ruleset_version = DEFAULT_WAVEFORM_RULESET_VERSION;
    const bool warn_unsafe = true;
    const int verbosity = 3;

    t_wtable wavetable;
    t_waveform waveform;
    waveform = {0, { {1, -2}, {3, -4}, {5, -6} } };
    wavetable.push_back(waveform);
#if 0    
    waveform = {1, { {11, -12}, {13, -14}, {15, -16} } };
    wavetable.push_back(waveform);
    waveform = {2, { {21, -22}, {23, -24}, {25, -26} } };
    wavetable.push_back(waveform);
#endif // 0
    
    ugd.getGridState(grid_state);

    ugd.enableMove(0, grid_state);
    
    result = ugd.configMotion(wavetable, grid_state, fpuset, soft_protection,
                              allow_uninitialized, ruleset_version, 
                              warn_unsafe, verbosity);

    //..........................................................................
    // Test executeMotion()
    bool sync_command = true;
    result = ugd.executeMotion(grid_state, fpuset, sync_command);
    
    //..........................................................................

    result = ugd.disconnect();

    //..........................................................................
}

//------------------------------------------------------------------------------
void UnprotectedGridDriverTester::test_initialize()
{

    
}

//------------------------------------------------------------------------------
void UnprotectedGridDriverTester::test_connect()
{

    
}

//------------------------------------------------------------------------------
void UnprotectedGridDriverTester::test_findDatum()
{


}

//------------------------------------------------------------------------------
void UnprotectedGridDriverTester::test_configMotion()
{
    
}

//------------------------------------------------------------------------------
void UnprotectedGridDriverTester::test_executeMotion()
{
    
}


//==============================================================================


} // namespace mpifps

