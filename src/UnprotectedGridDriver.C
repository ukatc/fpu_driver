// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-04-28  Created (translated from Python FpuGridDriver.py).
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME UnprotectedGridDriver.C
//
// This UnprotectedGridDriver class provides the main higher-level unprotected
// functionality for the grid driver. It also provides virtual, mostly-empty
// "hook" functions which are called from various places. The separate
// GridDriver class inherits from this UnprotectedGridDriver class and provides
// the FPU movement software protection functionality in its overrides of the
// hook functions.
//
////////////////////////////////////////////////////////////////////////////////



// ********** TODO: NOTE: This file (along with GridDriver.C) is Bart's work in
// progress for converting the classes and functions in FPUGridDriver.py from
// Python to C++.

#include <fcntl.h>
#include <algorithm>
#include <unistd.h>
#include <signal.h>
#include <new>
#include "UnprotectedGridDriver.h"
#include "DeviceLock.h"
#include "ethercan/FPUArray.h"

#ifdef DEBUG
#include <stdio.h>
#endif


namespace mpifps
{

//==============================================================================
// Abort signal handler functionality
//
// N.B. These variables and functions are file-scope globals to support the
// Linux signal handler function being a C function (rather than a C++ member
// function) to keep things simple.
static int sig_handler_signum = SIGINT;
// The following are atomics to make them thread-safe, because they are
// modified from the signal handler, which is called from a different thread.
// TODO: Would these variables be at risk of the C++ static initialization
// order problem / fiasco?   
static std::atomic<sighandler_t> sig_handler_original(nullptr);
static std::atomic<bool> sig_handler_triggered(false);
static std::atomic<bool> sig_handler_released(true);

static void sigHandlerRelease();

//------------------------------------------------------------------------------
static void sigHandlerFunc(int signum)
{
    if (signum == sig_handler_signum)
    {
        sigHandlerRelease();
        sig_handler_triggered = true;
    }
}

//------------------------------------------------------------------------------
static void sigHandlerRelease()
{
    if (!sig_handler_released)
    {
        signal(sig_handler_signum, sig_handler_original);
        sig_handler_released = true;
    }
}

//------------------------------------------------------------------------------
    
class SignalHandler
{
    // Context manager for handling a signal while waiting for command
    // completion.
    // NOTE: Only a single instance is allowed at a time.

    // TODO: Can't have this SignalHandler class directly in
    // UnprotectedGridDriver, because it’s a CONTROL-C mechanism for use in
    // the Boost,Python wrapper only - need to implement a GENERIC abort flag
    // mechanism in UnprotectedGridDriver

public:
    SignalHandler(int signum = SIGINT)
    {
        sig_handler_signum = signum;
        sig_handler_triggered = false;
        sig_handler_released = false;
        sig_handler_original = signal(sig_handler_signum, sigHandlerFunc);
    }

    bool wasTriggered()
    {
        bool was_triggered = sig_handler_triggered;
        sig_handler_triggered = false;
        return was_triggered;
    }
    
    ~SignalHandler()
    {
        sigHandlerRelease();
        sig_handler_triggered = false;
    }
};


//==============================================================================

// TODO: Temporary "warn()" placeholder for now - implement something better
#define warn(warnString)

//------------------------------------------------------------------------------
// TODO: Disabled these functions for now because not yet used, to stop build
// warnings - enable and implement fully if needed
#if 0
//------------------------------------------------------------------------------

static std::string get_logname(const std::string &basename,
                               const std::string &log_dir,
                               const std::string &timestamp);
static std::string make_logdir(const std::string &log_dir);


//------------------------------------------------------------------------------
std::string get_logname(const std::string &basename,
                        const std::string &log_dir,
                        const std::string &timestamp)
{
    // TODO: Convert from FpuGridDriver.py -> get_logname()

    // Temporary for now
    UNUSED_ARG(basename);
    UNUSED_ARG(log_dir);
    UNUSED_ARG(timestamp);

    if (timestamp == DEFAULT_START_TIMESTAMP)
    {

    }

    return std::string("");
}

//------------------------------------------------------------------------------
std::string make_logdir(const std::string &log_dir)
{

    // TODO: Convert from FpuGridDriver.py -> make_logdir() - not sure about
    // equivalent C++ path manipulations for Linux path handling yet

    // Temporary for now
    UNUSED_ARG(log_dir);

    return std::string("");
}

//------------------------------------------------------------------------------
#endif // 0
//------------------------------------------------------------------------------


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
    if ((nfpus > 0) && (nfpus <= MAX_NUM_POSITIONERS))
    {
        fpus_data.resize(nfpus);
        
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
    }
    else
    {
        config.num_fpus = 0;
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

    //................................
    // TODO: Temporary for now, to prevent build warnings
    UNUSED_ARG(log_dir);
    UNUSED_ARG(protection_logfile);
    UNUSED_ARG(control_logfile);
    UNUSED_ARG(tx_logfile);
    UNUSED_ARG(rx_logfile);
    UNUSED_ARG(start_timestamp);
    //................................

    if (initialize_was_called_ok)
    {
        return DE_INTERFACE_ALREADY_INITIALIZED;
    }

    if (config.num_fpus == 0)
    {
        return DE_INVALID_FPU_ID;
    }

    config.logLevel = logLevel;
    config.firmware_version_address_offset = firmware_version_address_offset;

    // TODO: Finish filling out this function from Python equivalent stuff

    // self.lock = threading.RLock()   // TODO: Adapt from Python - use e.g. 
                                       // pthread_mutex_lock()? (see EtherCANInterface.C)
                                       // AND need to unlock somehwere else, or would it be
                                       // done automatically due to RAII?


    //..........................................................................

    // TODO: Convert log file initialisation code from FpuGridDriver.py

    //int flags = O_CREAT | O_APPEND | O_WRONLY;
    //mode_t mode = 0644;  // Octal

    //std::string log_path = make_logdir(log_dir);


    //..........................................................................

    E_EtherCANErrCode ecan_result;

    _gd = new (std::nothrow) EtherCANInterface(config);
    if (_gd != nullptr)
    {
        // TODO: Call deInitializeInterface() or deInitialize() from destructor,
        // or elsewhere? BUT is already done from AsyncInterface destructor?
        ecan_result = _gd->initializeInterface();
        if ((ecan_result == DE_OK) || 
            (ecan_result == DE_INTERFACE_ALREADY_INITIALIZED))
        {
            initialize_was_called_ok = true;
        }
        else
        {
            delete _gd;

            // TODO: Check if also need to call _gd->deinitializeInterface() here
        }
    }
    else
    {
        // Fatal error
        ecan_result = DE_OUT_OF_MEMORY;
    }

    return ecan_result;
}

//------------------------------------------------------------------------------
bool UnprotectedGridDriver::initializedOk()
{
    return initialize_was_called_ok;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::connect(int ngateways,
                                const t_gateway_address gateway_addresses[])
{
    if (!initializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    // TODO: Also add gateway locking/unlocking code like the Python version
    // of this function does? (see below). If so then do NOT use my
    // DeviceLock.C/h WIP conversion from the Python-equivalent module
    // because too clunky - instead, use Linux named semaphores?

    E_EtherCANErrCode ecan_result = _gd->connect(ngateways, gateway_addresses);
    if (ecan_result == DE_OK)
    {
        ecan_result = _post_connect_hook();
    }
    return ecan_result;

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
    if (!initializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    return _gd->disconnect();
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
    
    for (int i = 0; i < config.num_fpus; i++)
    {
        if (fpuset[i] && (!gs.FPU_state[i].ping_ok))
        {
            pingset_ret[i] = true;
        }
    }    
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::setUStepLevel(int ustep_level,
                                                       t_grid_state &gs, 
                                                       const t_fpuset &fpuset)
{
    E_EtherCANErrCode ecan_result = checkInitializedAndFpuset(fpuset);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    return _gd->setUStepLevel(ustep_level, gs, fpuset);
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::setTicksPerSegment(unsigned long nticks,
                                                            t_grid_state &gs,
                                                            const t_fpuset &fpuset)
{
    E_EtherCANErrCode ecan_result = checkInitializedAndFpuset(fpuset);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    return _gd->setTicksPerSegment(nticks, gs, fpuset);
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::setStepsPerSegment(int min_steps,
                                                            int max_steps,
                                                            t_grid_state &gs,
                                                            const t_fpuset &fpuset) 
{
    E_EtherCANErrCode ecan_result = checkInitializedAndFpuset(fpuset);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    return _gd->setStepsPerSegment(min_steps, max_steps, gs, fpuset);
}

//------------------------------------------------------------------------------
E_GridState UnprotectedGridDriver::getGridState(t_grid_state &grid_state_ret)
{
    if (!initializedOk())
    {
        return GS_UNKNOWN;
    }

    return _gd->getGridState(grid_state_ret);
}

//------------------------------------------------------------------------------
int UnprotectedGridDriver::getNumFpus()
{
    return config.num_fpus;
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

    E_EtherCANErrCode ecan_result = checkInitializedAndFpuset(fpuset);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    // TODO: Add C++/Linux equivalent of Python version's "with self.lock" here 

    // Make temporary local copy of search modes to allow local modification
    t_datum_search_flags search_modes;
    for (int i = 0; i < MAX_NUM_POSITIONERS; i++)
    {
        search_modes[i] = orig_search_modes[i];
    }

    if (soft_protection)
    {
        // Check whether datum search is safe, adjusting search_modes
        ecan_result = _allow_find_datum_hook(gs, search_modes, selected_arm,
                                             fpuset, support_uninitialized_auto);
        if (ecan_result != DE_OK)    
        {
            return ecan_result;
        }

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
    ecan_result = _start_find_datum_hook(gs, search_modes, selected_arm, fpuset,
                                         initial_positions, soft_protection);
    if (ecan_result != DE_OK)    
    {
        return ecan_result;
    }

    t_grid_state prev_gs;
    _gd->getGridState(prev_gs);

    ecan_result = _gd->startFindDatum(gs, search_modes, selected_arm, timeout,
                                      count_protection, &fpuset);
    if (ecan_result != DE_OK)
    {
        // We cancel the datum search altogether, so we can reset positions
        // to old value
        // Note: We do NOT check and return the _cancel_find_datum_hook()
        // return value here, because the startFindDatum() return value above
        // is far more important, and _cancel_find_datum_hook() isn't very 
        // likely to fail anyway
        _cancel_find_datum_hook(gs, fpuset, initial_positions);

        return ecan_result;
    }

    double time_interval = 0.1;
    sleepSecs(time_interval);
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
        ecan_result = _gd->waitFindDatum(gs, time_interval,
                                         wait_find_datum_finished, &fpuset);
        is_ready = (ecan_result != DE_WAIT_TIMEOUT);
        finished_ok = (ecan_result == DE_OK);
    }

    if (!finished_ok)
    {
        sleepSecs(time_interval);

        E_EtherCANErrCode ping_if_needed_result = pingIfNeeded(gs, fpuset);
        if (ping_if_needed_result != DE_OK)
        {
            return ping_if_needed_result;
        }
    }

    E_EtherCANErrCode finished_func_result = 
                _finished_find_datum_hook(prev_gs, gs, search_modes, fpuset,
                                          !finished_ok, initial_positions,
                                          selected_arm);
    if (finished_func_result != DE_OK)
    {
        return finished_func_result;
    }

    if (was_aborted)
    {

        // TODO: See Python equivalent - not appropriate to abort etc? (also
        // see comments further up in this function)

    }

    return ecan_result;
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
    E_EtherCANErrCode ecan_result = check_fpuset(fpuset);
    if (ecan_result == DE_OK)
    {
        ecan_result = _gd->pingFPUs(gs, fpuset);
    }

    return ecan_result;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::pingFPUs(t_grid_state &gs,
                                                  const t_fpuset &fpuset)
{
    E_EtherCANErrCode ecan_result = checkInitializedAndFpuset(fpuset);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    // Communicates with all FPUs and queries their status
    return _pingFPUs(gs, fpuset);
}


//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::resetFPUs(t_grid_state &gs,
                                                   const t_fpuset &fpuset)
{
    E_EtherCANErrCode ecan_result = checkInitializedAndFpuset(fpuset);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    // TODO: Add C++/Linux equivalent of Python version's "with self.lock" here 

    t_grid_state prev_gs;
    _gd->getGridState(prev_gs);
    
    ecan_result = _gd->resetFPUs(gs, fpuset);

    // TODO: Only execute the remaining code below if ecan_result from the
    // resetFPUs() call above returns DE_OK? (check WRT Python version, and
    // other more recently-converted similar C++ functions, e.g.
    // resetStepCounters() below)

    updateErrorCountersForFpuSet(prev_gs, gs, fpuset);

    // Clear FPU waveforms
    for (size_t fpu_id = 0; fpu_id < fpus_data.size(); fpu_id++)
    {
        fpus_data[fpu_id].db.last_waveform.clear();
    }
    
    // Wait for FPUs to become active (N.B. values adapted from original
    // Python version of this function)
    sleepSecs(0.1 * 24.0);

    ecan_result = _reset_hook(prev_gs, gs, fpuset);

    return ecan_result;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::resetStepCounters(long new_alpha_steps,
                                                           long new_beta_steps,
                                                           t_grid_state &gs,
                                                           const t_fpuset &fpuset)
{
    E_EtherCANErrCode ecan_result = checkInitializedAndFpuset(fpuset);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    // TODO: Add C++/Linux equivalent of Python version's "with self.lock" here

    t_grid_state prev_gs;
    getGridState(prev_gs);

    ecan_result = _gd->resetStepCounters(new_alpha_steps, new_beta_steps,
                                         gs, fpuset);

    updateErrorCountersForFpuSet(prev_gs, gs, fpuset);

    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    double alpha_target = (((double)new_alpha_steps) / StepsPerDegreeAlpha) +
                          config.alpha_datum_offset;
    double beta_target = ((double)new_beta_steps) / StepsPerDegreeBeta;

    _reset_counter_hook(alpha_target, beta_target, prev_gs, gs, fpuset);
    return DE_OK;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::readRegister(uint16_t address,
                                                      t_grid_state &gs,
                                                      const t_fpuset &fpuset)
{
    E_EtherCANErrCode ecan_result = checkInitializedAndFpuset(fpuset);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    t_grid_state prev_gs;
    _gd->getGridState(prev_gs);

    ecan_result = _gd->readRegister(address, gs, fpuset);

    updateErrorCountersForFpuSet(prev_gs, gs, fpuset);
    
    return ecan_result;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver:: getDiagnostics(t_grid_state &gs,
                                                         const t_fpuset &fpuset,
                                                         std::string &string_ret)
{
    E_EtherCANErrCode ecan_result = checkInitializedAndFpuset(fpuset);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    static const struct
    {
        const char *name;
        uint16_t address;
    } reg_defs[] =
    {
        { "sstatus2",   0x00001E },
        { "sstatus3",   0x00001F },
        { "sstatus4",   0x000020 },
        { "sstatus5",   0x000021 },
        { "intflags",   0x000022 },
        { "stateflags", 0x000023 }
    }; 

    // TODO: Adjust the following so that the fields are all the same width so
    // that neat columns are produced, and also change so that the register
    // addresses and values are displayed in hex

    string_ret = "  RegName    RegAddress:";
    for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
    {
        if (fpuset[fpu_id])
        {
            string_ret += "  FPU[" + std::to_string(fpu_id) + "] ";
        }
    }
    string_ret += "\n";

    string_ret += "  --------   -----------";
    for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
    {
        if (fpuset[fpu_id])
        {
            string_ret += "  ------ ";
        }
    }
    string_ret += "\n";

    for (int i = 0; i < (sizeof(reg_defs) / sizeof(reg_defs[0]));
         i++)
    {
        E_EtherCANErrCode ecan_result = readRegister(reg_defs[i].address,
                                                     gs, fpuset);
        if (ecan_result == DE_OK)   
        {
            string_ret += std::string(reg_defs[i].name) + "        " +
                          std::to_string(reg_defs[i].address);
            for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
            {
                if (fpuset[fpu_id])
                {
                    string_ret += "    " + 
                        std::to_string(gs.FPU_state[fpu_id].register_value) + " ";
                }
            }
        }
        else
        {
            string_ret += "ERROR: readRegister() failed";
        }
        string_ret += "\n";
    }
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::getFirmwareVersion(t_grid_state &gs,
                                                        const t_fpuset &fpuset)
{
    E_EtherCANErrCode ecan_result = checkInitializedAndFpuset(fpuset);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    t_grid_state prev_gs;
    _gd->getGridState(prev_gs);

    ecan_result = _gd->getFirmwareVersion(gs, fpuset);

    updateErrorCountersForFpuSet(prev_gs, gs, fpuset);

    return ecan_result;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::readSerialNumbers(t_grid_state &gs,
                                                           const t_fpuset &fpuset)
{
    E_EtherCANErrCode ecan_result = checkInitializedAndFpuset(fpuset);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    t_grid_state prev_gs;
    _gd->getGridState(prev_gs);

    ecan_result = _gd->readSerialNumbers(gs, fpuset);

    updateErrorCountersForFpuSet(prev_gs, gs, fpuset);

    return ecan_result;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::writeSerialNumber(int fpu_id,
                                                    const char *serial_number,
                                                    t_grid_state &gs)
{
    if (!initializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    // TODO: Add C++/Linux equivalent of Python version's "with self.lock" here 

    t_grid_state prev_gs;
    _gd->getGridState(prev_gs);

    t_fpuset fpuset;
    createFpuSetForSingleFpu(fpu_id, fpuset);
    
    E_EtherCANErrCode ecan_result = _gd->writeSerialNumber(fpu_id,
                                                           serial_number, gs);
    if (ecan_result == DE_OK)
    {
        ecan_result = _gd->readSerialNumbers(gs, fpuset);
    }

    updateErrorCountersForFpuSet(prev_gs, gs, fpuset);

    return ecan_result;
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

    When the 'protected' flag is set to false, bypass all hardware protection
    checks, which will allow to move a collided or uncalibrated FPU (even if
    the movement might damage the hardware).
    */

    // TODO
    UNUSED_ARG(verbosity);

    // TODO: NOTE: UnprotectedGridDriver::_post_config_motion_hook() DOES also
    // have a body (see UnprotectedGridDriver.h), in addition to
    // GridDriver::_post_config_motion_hook() - check it

    // TODO: Sort out return values

    E_EtherCANErrCode ecan_result = checkInitializedAndFpuset(fpuset);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    // TODO: Add C++/Linux equivalent of Python version's "with self.lock" here 

    // We make a copy of the wavetable to make sure no side effects are
    // visible to the caller
    t_wtable wtable = wavetable;

    // Delete all wtable elements not specified in fpuset
    // N.B. This isn't very efficient because we might be deleting the
    // std::vector elements from the middle - but will work OK
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

    ecan_result = _pre_config_motion_hook(wtable, gs, fpuset, wmode);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

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

    ecan_result = _gd->configMotion(wtable, gs, fpuset, allow_uninitialized,
                                    ruleset_version);
    // TODO: Check for the various expected result values - might some
    // non-DE_OK ones might actually be OK?
    if (ecan_result != DE_OK)
    {
        update_config = true;
    }

    if (update_config)
    {
        // TODO: Much of the following code is inefficient because iterates
        // through arrays/lists many times - but should work OK. This approach
        // is currently taken in order to conform with the existing t_wtable
        // data structure already defined.

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
                // Set the FPU's last_waveform
                fpus_data[fpu_id].db.last_waveform = it->steps;
            }
            else
            {
                wtable.erase(it);
            }
            
            _update_error_counters(fpus_data[fpu_id].db.counters,
                                   prev_gs.FPU_state[fpu_id],
                                   gs.FPU_state[fpu_id]);
        }

        ecan_result = _post_config_motion_hook(wtable, gs, fpuset);
    }

    // TODO: Is this return variable correct? (does correspond to equivalent 
    // Python code)
    return ecan_result;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::executeMotion(t_grid_state &gs,
                                                       const t_fpuset &fpuset,
                                                       bool sync_command)
{
    E_EtherCANErrCode ecan_result = checkInitializedAndFpuset(fpuset);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    // TODO: Add C++/Linux equivalent of Python version's "with self.lock" here

    // Wait a short moment to avoid spurious collision
    t_fpu_positions initial_positions;
    ecan_result = _start_execute_motion_hook(gs, fpuset, initial_positions);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }
    sleepSecs(0.1);
    t_grid_state prev_gs;
    _gd->getGridState(prev_gs);
    sleepSecs(0.1);
    ecan_result = _gd->startExecuteMotion(gs, fpuset, sync_command);
    // TODO: The ecan_result logic here might not be the same as the Python
    // equivalent - check this
    if (ecan_result != DE_OK)
    {
        // Note: Do NOT want to return the _cancel_execute_motion_hook() return
        // value here, because more interested in the startExecuteMotion()
        // ecan_result value above
        _cancel_execute_motion_hook(gs, fpuset, initial_positions);
        return ecan_result;
    }

    double time_interval = 0.1;
    bool is_ready = false;
    //bool was_aborted = false;
    bool refresh_state = false;
    // TODO: The ecan_result logic here might not be the same as the Python
    // equivalent - check this
    while (!is_ready)
    {
        bool wait_execute_motion_finished = false;
        ecan_result = _gd->waitExecuteMotion(gs, time_interval,
                                             wait_execute_motion_finished,
                                             fpuset);
        
        // TODO: Python version can be interrupted from Linux signals here,
        // but this isn't appropriate for C++ version because it wlll be
        // inside ESO driver?

        // (The following check was brought in from 
        // ethercanif.C -> WrapEtherCANInterface::wrap_waitExecuteMotion())
        if (!wait_execute_motion_finished && (ecan_result == DE_OK))
        {
            ecan_result = DE_WAIT_TIMEOUT;
        }

        is_ready = (ecan_result != DE_WAIT_TIMEOUT);
    }

    if (ecan_result != DE_OK)
    {
        refresh_state = true;
    }

    // This is skipped in case of a SocketFailure, for example
    if ((ecan_result == DE_OK) || refresh_state)
    {
        // Execute a ping to update positions
        // (this is only needed for protocol version 1)  <== TODO: Check this
        t_grid_state move_gs;
        _gd->getGridState(move_gs);

        sleepSecs(0.1);

        E_EtherCANErrCode ping_if_needed_result = pingIfNeeded(gs, fpuset);
        if (ping_if_needed_result != DE_OK)
        {
            return ping_if_needed_result;
        }

        // The following hook will narrow down the recorded intervals of
        // positions
        E_EtherCANErrCode post_func_result = 
                    _post_execute_motion_hook(gs, prev_gs, move_gs, fpuset);
        if (post_func_result != DE_OK)
        {
            return post_func_result;
        }
    }

    // TODO: Original Python code - need to do anything with this here?
    //if was_aborted:
    //raise MovementError("executeMotion was aborted by SIGINT")

    return ecan_result;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::abortMotion(t_grid_state &gs,
                                                     const t_fpuset &fpuset,
                                                     bool sync_command)
{
    E_EtherCANErrCode ecan_result = checkInitializedAndFpuset(fpuset);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    // This must not use locking - it can be sent from any thread by design
    // TODO: Check and understand this further (relative to the Python version)

    t_grid_state prev_gs;
    _gd->getGridState(prev_gs);

    ecan_result = _gd->abortMotion(gs, fpuset, sync_command);

    updateErrorCountersForFpuSet(prev_gs, gs, fpuset);

    return ecan_result;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::freeBetaCollision(int fpu_id,
                                                E_REQUEST_DIRECTION direction,
                                                t_grid_state &gs,
                                                bool soft_protection)
{
    if (!initializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    // TODO: Add C++/Linux equivalent of Python version's "with self.lock" here
    E_EtherCANErrCode ecan_result;
    ecan_result = _pre_free_beta_collision_hook(fpu_id, direction, gs,
                                                soft_protection);
    if (ecan_result != DE_OK)    
    {
        return ecan_result;
    }

    t_grid_state prev_gs;
    getGridState(prev_gs);

    ecan_result = _gd->freeBetaCollision(fpu_id, direction, gs);

    t_fpuset fpuset;
    createFpuSetForSingleFpu(fpu_id, fpuset);
    updateErrorCountersForFpuSet(prev_gs, gs, fpuset);

    E_EtherCANErrCode post_result = _post_free_beta_collision_hook(fpu_id,
                                                                   direction,
                                                                   gs);
    // N.B. freeBetaCollision()'s return value is a priority
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }
    return post_result;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::enableBetaCollisionProtection(
                                                             t_grid_state &gs)
{
    if (!initializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    t_grid_state prev_gs;
    _gd->getGridState(prev_gs);

    E_EtherCANErrCode ecan_result = _gd->enableBetaCollisionProtection(gs);

    t_fpuset fpuset;
    createFpuSetForNumFpus(config.num_fpus, fpuset);
    updateErrorCountersForFpuSet(prev_gs, gs, fpuset);

    return ecan_result;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::freeAlphaLimitBreach(int fpu_id,
                                                E_REQUEST_DIRECTION direction,
                                                t_grid_state &gs,
                                                bool soft_protection)
{
    if (!initializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    // TODO: Add C++/Linux equivalent of Python version's "with self.lock" here
    E_EtherCANErrCode ecan_result;
    ecan_result = _pre_free_alpha_limit_breach_hook(fpu_id, direction, gs,
                                                    soft_protection);
    if (ecan_result != DE_OK)    
    {
        return ecan_result;
    }

    t_grid_state prev_gs;
    getGridState(prev_gs);

    ecan_result = _gd->freeAlphaLimitBreach(fpu_id, direction, gs);

    t_fpuset fpuset;
    createFpuSetForSingleFpu(fpu_id, fpuset);
    updateErrorCountersForFpuSet(prev_gs, gs, fpuset);

    E_EtherCANErrCode post_result = _post_free_alpha_limit_breach_hook(fpu_id,
                                                                       direction,
                                                                       gs);
    // N.B. freeAlphaLimitBreach()'s return value is a priority
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }
    return post_result;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::enableAlphaLimitProtection(
                                                            t_grid_state &gs)
{
    if (!initializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    t_grid_state prev_gs;
    _gd->getGridState(prev_gs);

    E_EtherCANErrCode ecan_result = _gd->enableAlphaLimitProtection(gs);

    t_fpuset fpuset;
    createFpuSetForNumFpus(config.num_fpus, fpuset);
    updateErrorCountersForFpuSet(prev_gs, gs, fpuset);

    return ecan_result;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::reverseMotion(t_grid_state &gs,
                                                       const t_fpuset &fpuset,
                                                       bool soft_protection)
{
    E_EtherCANErrCode ecan_result = checkInitializedAndFpuset(fpuset);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    // TODO: Add C++/Linux equivalent of Python version's "with self.lock" here

    t_wtable wtable;
    buildWtableFromLastWaveforms(fpuset, wtable);

    Range wmode;
    if (soft_protection)
    {
        wmode = Range::Error;
    }
    else
    {
        wmode = Range::Warn;
    }

    ecan_result = _pre_reverse_motion_hook(wtable, gs, fpuset, wmode);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    t_grid_state prev_gs;
    _gd->getGridState(prev_gs);

    ecan_result = _gd->reverseMotion(gs, fpuset);

    updateErrorCountersForFpuSet(prev_gs, gs, fpuset);

    E_EtherCANErrCode post_result = _post_reverse_motion_hook(wtable, gs,
                                                              fpuset);
    // Prioritise _gd->reverseMotion() return code
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }
    return post_result;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::repeatMotion(t_grid_state &gs,
                                                      const t_fpuset &fpuset,
                                                      bool soft_protection)
{
    E_EtherCANErrCode ecan_result = checkInitializedAndFpuset(fpuset);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    // TODO: Add C++/Linux equivalent of Python version's "with self.lock" here

    t_wtable wtable;
    buildWtableFromLastWaveforms(fpuset, wtable);

    Range wmode;
    if (soft_protection)
    {
        wmode = Range::Error;
    }
    else
    {
        wmode = Range::Warn;
    }

    ecan_result = _pre_repeat_motion_hook(wtable, gs, fpuset, wmode);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    t_grid_state prev_gs;
    _gd->getGridState(prev_gs);
    
    ecan_result = _gd->repeatMotion(gs, fpuset);

    updateErrorCountersForFpuSet(prev_gs, gs, fpuset);

    E_EtherCANErrCode post_result = _post_repeat_motion_hook(wtable, gs,
                                                             fpuset);
    // Prioritise _gd->repeatMotion() return code
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }
    return post_result;
}

//------------------------------------------------------------------------------
void UnprotectedGridDriver::buildWtableFromLastWaveforms(const t_fpuset &fpuset,
                                                         t_wtable &wtable_ret)
{
    wtable_ret.clear();
    for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
    {
        t_waveform_steps &last_waveform = fpus_data[fpu_id].db.last_waveform;
        if (fpuset[fpu_id] && (last_waveform.size() != 0))
        {
            wtable_ret.push_back({(int16_t)fpu_id, last_waveform});
        }
    }
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::countedAngles(t_grid_state &gs,
                                                       const t_fpuset &fpuset,
                                                       t_fpus_angles &fpus_angles_ret,
                                                       bool show_uninitialized)
{
    // Returns alpha and beta angles (in degrees) for the given set of FPUs
    // based upon the motor step count relative to datum. The angles are only
    // valid if the FPU has been referenced. Set show_uninitialized to true to
    // return the step count angles regardless of the state of the referenced
    // flag (it is up to the caller to decide if the counts can still be
    // trusted).

    E_EtherCANErrCode ecan_result = checkInitializedAndFpuset(fpuset);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    t_grid_state prev_gs;
    _gd->getGridState(prev_gs);

    ecan_result = _pingFPUs(gs, fpuset);

    updateErrorCountersForFpuSet(prev_gs, gs, fpuset);

    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    fpus_angles_ret.clear();
    t_fpus_angles fpus_angles_temp;
    list_angles(gs, config.num_fpus, fpus_angles_temp,
                config.alpha_datum_offset);
    for (size_t i = 0; i < fpus_angles_temp.size(); i++)
    {
        if (fpuset[fpus_angles_temp[i].first])
        {
            fpus_angles_ret.push_back(fpus_angles_temp[i]);
        }
    }

    return DE_OK;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::lockFPU(int fpu_id, t_grid_state &gs)
{
    if (!initializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    // TODO: Add C++/Linux equivalent of Python version's "with self.lock" here
    t_grid_state prev_gs;
    _gd->getGridState(prev_gs);

    E_EtherCANErrCode ecan_result = _gd->lockFPU(fpu_id, gs);

    t_fpuset fpuset;
    createFpuSetForSingleFpu(fpu_id, fpuset);
    updateErrorCountersForFpuSet(prev_gs, gs, fpuset);

    return ecan_result;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::unlockFPU(int fpu_id, t_grid_state &gs)
{
    if (!initializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    // TODO: Add C++/Linux equivalent of Python version's "with self.lock" here
    t_grid_state prev_gs;
    _gd->getGridState(prev_gs);

    E_EtherCANErrCode ecan_result = _gd->unlockFPU(fpu_id, gs);

    t_fpuset fpuset;
    createFpuSetForSingleFpu(fpu_id, fpuset);
    updateErrorCountersForFpuSet(prev_gs, gs, fpuset);

    return ecan_result;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::enableMove(int fpu_id, t_grid_state &gs)
{
    // TODO: Add C++/Linux equivalent of Python version's "with self.lock" here

    if (!initializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    t_grid_state prev_gs;
    _gd->getGridState(prev_gs);

    E_EtherCANErrCode ecan_result = _gd->enableMove(fpu_id, gs);

    // TODO: The following line was in original Python FpuGridDriver.enableMove(),
    // but not needed here?
    // status = gs.FPU[fpu_id].last_status

    t_fpuset fpuset;
    createFpuSetForSingleFpu(fpu_id, fpuset);
    updateErrorCountersForFpuSet(prev_gs, gs, fpuset);

    return ecan_result;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::checkIntegrity(t_grid_state &gs,
                                                        const t_fpuset &fpuset)
{
    E_EtherCANErrCode ecan_result = checkInitializedAndFpuset(fpuset);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    t_grid_state prev_gs;
    _gd->getGridState(prev_gs);

    ecan_result = _gd->checkIntegrity(gs, fpuset);

    updateErrorCountersForFpuSet(prev_gs, gs, fpuset);

    return ecan_result;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::checkInitializedAndFpuset(
                                                        const t_fpuset &fpuset)
{
    if (!initializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    return check_fpuset(fpuset);
}

//------------------------------------------------------------------------------
void UnprotectedGridDriver::updateErrorCountersForFpuSet(const t_grid_state &prev_gs,
                                                         const t_grid_state &gs,
                                                         const t_fpuset &fpuset,
                                                         bool datum_cmd)
{
    for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
    {
        if (fpuset[fpu_id])
        {
            _update_error_counters(fpus_data[fpu_id].db.counters,
                                   prev_gs.FPU_state[fpu_id],
                                   gs.FPU_state[fpu_id], datum_cmd);
        }
    }
}

//------------------------------------------------------------------------------
void UnprotectedGridDriver::sleepSecs(double seconds)
{
    static const double microsecs_in_1_sec = 1000000.0;
    usleep((useconds_t)(seconds * microsecs_in_1_sec));
}

//------------------------------------------------------------------------------
E_EtherCANErrCode UnprotectedGridDriver::check_fpuset(const t_fpuset &fpuset)
{
    // Check that config.num_fpus is within range
    if ((config.num_fpus < 0) || (config.num_fpus >= MAX_NUM_POSITIONERS))
    {
        // TODO: Any better error return code for this? DE_INVALID_CONFIG?
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
void UnprotectedGridDriver::createFpuSetForSingleFpu(int fpu_id,
                                                     t_fpuset &fpuset_ret)
{
    for (int i = 0; i < MAX_NUM_POSITIONERS; i++)
    {
        fpuset_ret[i] = false;
    }
    if ((fpu_id >= 0) && (fpu_id < MAX_NUM_POSITIONERS))
    {
        fpuset_ret[fpu_id] = true;
    }
}

//------------------------------------------------------------------------------
void UnprotectedGridDriver::createFpuSetForNumFpus(int num_fpus,
                                                   t_fpuset &fpuset_ret)
{
    if (num_fpus >= MAX_NUM_POSITIONERS)
    {
        num_fpus = MAX_NUM_POSITIONERS;
    }

    for (int i = 0; i < MAX_NUM_POSITIONERS; i++)
    {
        fpuset_ret[i] = false;
    }
    for (int i = 0; i < num_fpus; i++)
    {
        fpuset_ret[i] = true;
    }
}

//==============================================================================


} // namespace mpifps

