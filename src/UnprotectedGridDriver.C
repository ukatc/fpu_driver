// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-04-28  Created (translated from Python FpuGridDriver.py).
// bwillemse 2021-03-26  Modified for new non-contiguous FPU IDs and CAN mapping.
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
#include <new>
#include <string>
#include <fstream>
#include <set>
#include "UnprotectedGridDriver.h"
#include "DeviceLock.h"
#include "ethercan/FPUArray.h"
#include "FPUConstants.h"

#ifdef DEBUG
#include <stdio.h>
#endif


namespace mpifps
{

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


static std::atomic<bool> abort_motion_pending(false);

//------------------------------------------------------------------------------
void gridDriverAbortDuringFindDatumOrExecuteMotion(void)
{
    // Triggers the aborting of FPU motion during findDatum() or
    // executeMotion() if either of them are currently in the process of moving
    // FPUs.
    // NOTE: This function will be called from a DIFFERENT THREAD from the
    // thread on which the grid driver is running, so a thread-safe atomic
    // variable is used.
    abort_motion_pending = true;
}

#ifdef FLEXIBLE_CAN_MAPPING
//------------------------------------------------------------------------------
E_EtherCANErrCode gridDriverReadCanMapCsvFile(const std::string &csv_file_path,
                                              GridCanMap &grid_can_map_ret,
                                      CanMapCsvFileErrorInfo &error_info_ret)
{
    // Reads the FPU ID and CAN map data from a grid driver CAN map CSV file.
    // Arguments / return values:
    //   - csv_file_path: Must be of the general form e.g. "/moons/test_jig.csv"
    //   - If the CSV file was successfully parsed then returns DE_OK, and
    //     grid_can_map_ret will contain the FPU ID and CAN mapping list
    //   - If an error occurs then one of the following error codes is
    //     returned, grid_can_map_ret will be of size 0, and the values in
    //     error_info_ret will give more information for the particular error
    //     wherever relevant (including the line number of the error in the
    //     CSV file):
    //       - DE_RESOURCE_ERROR if the CSV file couldn't be opened for some
    //         reason
    //       - DE_INVALID_NUM_PARAMS if incorrect number of fields on a line
    //       - DE_INVALID_PAR_VALUE if any of the fields on a line couldn't be
    //         converted to their required type
    //       - DE_NO_FPUS_DEFINED if no FPU ID / CAN route lines were
    //         successfully found
    //       - DE_INVALID_FPU_ID / DE_INVALID_GATEWAY_ID /
    //         DE_INVALID_CAN_BUS_ID / DE_INVALID_CAN_ID if an out-of-range
    //         invalid value is found
    //       - DE_DUPLICATE_FPU_ID / DE_DUPLICATE_CAN_ROUTE if a duplicate
    //         one of these is found

    grid_can_map_ret.clear();

    E_EtherCANErrCode ecan_result = DE_ERROR_UNKNOWN;

    // Attempt to open file
    std::ifstream csv_file_stream;
    csv_file_stream.open(csv_file_path);
    if (csv_file_stream.good())
    {
        ecan_result = DE_OK;
    }
    else
    {
        return DE_RESOURCE_ERROR;
    }

    int current_line_num = 0;
    std::string line_str;
    std::set<int> fpu_ids_temp;                 // | Temporary sets used for
    std::set<uint32_t> can_routes_uints_temp;   // | duplicate checking

    // Parse and check each line's data items and add to FPU ID / CAN route list
    while (std::getline(csv_file_stream, line_str))
    {
        current_line_num++;

        if (line_str.empty())
        {
            continue;
        }

        std::stringstream line_stream(line_str);

        // Get the line's items into a vector of strings
        // ***************** TODO: Improve this to support the required delimiter(s) - spaces,
        // tabs, commas?
        std::string line_item_str;
        std::vector<std::string> line_item_strs;
        while (line_stream >> line_item_str)
        {
            line_item_strs.push_back(line_item_str);
        }

        int fpu_id = 0;
        FPUArray::t_bus_address can_route = { 0, 0, 0 };
        if (line_item_strs.size() == 4)
        {
            //..................................................................
            // Extract the line's fields
            try
            {
                fpu_id = std::stoi(line_item_strs[0]);
                can_route.gateway_id = std::stoi(line_item_strs[1]);
                can_route.bus_id = std::stoi(line_item_strs[2]);
                can_route.can_id = std::stoi(line_item_strs[3]);
            }
            catch(const std::exception &e)
            {
                // Error - could not convert a field for some reason
                ecan_result = DE_INVALID_PAR_VALUE;
            }

            //..................................................................
            // Check the field values' ranges
            if (ecan_result == DE_OK)
            {
                if ((fpu_id < 0) || (fpu_id >= FPU_ID_BROADCAST_BASE))
                {
                    ecan_result = DE_INVALID_FPU_ID;
                }
                else if ((can_route.gateway_id < 0) ||
                         (can_route.gateway_id >= MAX_NUM_GATEWAYS))
                {
                    ecan_result = DE_INVALID_GATEWAY_ID;
                }
                else if ((can_route.bus_id < 0) ||
                         (can_route.bus_id >= BUSES_PER_GATEWAY))
                {
                    ecan_result = DE_INVALID_CAN_BUS_ID;
                }
                else if ((can_route.can_id < 1) ||
                         (can_route.can_id > FPUS_PER_BUS))
                {
                    ecan_result = DE_INVALID_CAN_ID;
                }
            }

            //..................................................................
            // If fpu_id and CAN route are not duplicates then add the FPU ID /
            // CAN route item to the list
            if (ecan_result == DE_OK)
            {
                uint32_t can_route_uint =
                                (((uint32_t)can_route.gateway_id) << 16) |
                                (((uint32_t)can_route.bus_id) << 8) |
                                ((uint32_t)can_route.can_id);
                if (!fpu_ids_temp.insert(fpu_id).second)
                {
                    ecan_result = DE_DUPLICATE_FPU_ID;
                }
                else if (!can_routes_uints_temp.insert(can_route_uint).second)
                {
                    ecan_result = DE_DUPLICATE_CAN_ROUTE;
                }
                else
                {
                    // Add the FPU ID / CAN route item to the list
                    std::pair<int, FPUArray::t_bus_address> fpu_can_route;
                    fpu_can_route.first = fpu_id;
                    fpu_can_route.second = can_route;
                    grid_can_map_ret.push_back(fpu_can_route);
                }
            }

            //..................................................................
        }
        else
        {
            ecan_result = DE_INVALID_NUM_PARAMS;
        }

        if (ecan_result != DE_OK)
        {
            error_info_ret.line_number = current_line_num;
            error_info_ret.fpu_id = fpu_id;
            error_info_ret.can_route = can_route;
            break;
        }
    }

    if (grid_can_map_ret.size() == 0)
    {
        ecan_result = DE_NO_FPUS_DEFINED;
    }

    if (ecan_result != DE_OK)
    {
        grid_can_map_ret.clear();
    }

    if (csv_file_stream.is_open())
    {
        csv_file_stream.close();
    }

    return ecan_result;
}

//------------------------------------------------------------------------------
#endif // FLEXIBLE_CAN_MAPPING


//==============================================================================

UnprotectedGridDriver::UnprotectedGridDriver(
    // NOTE: Boost.Python only allows up to 14 function arguments in
    // function wrappers, so need to keep number of arguments within this
#ifndef FLEXIBLE_CAN_MAPPING // NOT FLEXIBLE_CAN_MAPPING
    int nfpus,
#endif // NOT FLEXIBLE_CAN_MAPPING
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
#ifdef FLEXIBLE_CAN_MAPPING
    fpus_data.resize(MAX_NUM_POSITIONERS);
#else // NOT FLEXIBLE_CAN_MAPPING
    if ((nfpus > 0) && (nfpus <= MAX_NUM_POSITIONERS))
    {
        fpus_data.resize(nfpus);
        
        config.num_fpus = nfpus;
#endif // NOT FLEXIBLE_CAN_MAPPING
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
#ifndef FLEXIBLE_CAN_MAPPING // NOT FLEXIBLE_CAN_MAPPING
    }
    else
    {
        config.num_fpus = 0;
    }
#endif // NOT FLEXIBLE_CAN_MAPPING
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
#ifdef FLEXIBLE_CAN_MAPPING
                                const std::string &can_map_file_path,
#endif // FLEXIBLE_CAN_MAPPING
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

#ifndef FLEXIBLE_CAN_MAPPING // NOT FLEXIBLE_CAN_MAPPING
    if (config.num_fpus <= 0)
    {
        return DE_INVALID_FPU_ID;
    }
#endif // NOT FLEXIBLE_CAN_MAPPING

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

    E_EtherCANErrCode ecan_result = DE_ERROR_UNKNOWN;

#ifdef FLEXIBLE_CAN_MAPPING

    // TODO here: Try opening can_map_file_path:
    //   - If opens OK then parse it, check for ranges/duplicates etc, and
    //     populate grid_can_map from it
    //   - Any need to sort it into FPU ID order? Any advantage to this? (I
    //     don't think so?)
    //   - If doesn't open OK then abort with appropriate error code - add
    //     further DE_XXX error code(s) for errors relating to the CAN map
    //     file?

    //*****************
    //  TODO: Temporary for testing
    GridCanMap grid_can_map =
    {
        // FPU ID   gateway_id  bus_id  can_id

        {  28,     { 0,          0,      1} },
        //{  1,     { 0,          0,      2} },  **************************
        {  45,     { 0,          0,      3} }
    };
    //*****************

    // Create fpu_id_list and populate it from the FPU IDs in grid_can_map
    std::vector<int> fpu_id_list;
    for (size_t i = 0; i < grid_can_map.size(); i++)
    {
        fpu_id_list.push_back(grid_can_map[i].first);
    }

    ecan_result = config.initFpuIdList(fpu_id_list);
    if (ecan_result == DE_OK)
    {
        _gd = new (std::nothrow) EtherCANInterface(config, grid_can_map);
    }
    else
    {
        return ecan_result;
    }
#else // NOT FLEXIBLE_CAN_MAPPING
    _gd = new (std::nothrow) EtherCANInterface(config);
#endif // NOT FLEXIBLE_CAN_MAPPING
    if (_gd != nullptr)
    {
        ecan_result = _gd->initializeInterface();
        if ((ecan_result == DE_OK) || 
            (ecan_result == DE_INTERFACE_ALREADY_INITIALIZED))
        {
            initialize_was_called_ok = true;
        }
        else
        {
            delete _gd;
            _gd = nullptr;
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

    // TODO: Add C++/Linux equivalent of Python version's "with self.lock" here 

    // TODO: Also add gateway locking/unlocking code like the Python version
    // of this function does? (see below). If so then do NOT use my
    // DeviceLock.C/h WIP conversion from the Python-equivalent module
    // because too clunky - instead, use Linux named semaphores?
    //   self.locked_gateways = [] # this del's and releases any previously acquired locks
    //   for gw in address_list:
    //      groupname = os.environ.get("MOONS_GATEWAY_ACCESS_GROUP","moons")
    //      # acquire a unique inter-process lock for each gateway IP
    //      dl = devicelock.DeviceLock('ethercan-gateway@%s:%i' % (gw.ip, gw.port), groupname)
    //      self.locked_gateways.append(dl)

    E_EtherCANErrCode ecan_result = _gd->connect(ngateways, gateway_addresses);
    if (ecan_result == DE_OK)
    {
        ecan_result = _post_connect_hook();
    }
    return ecan_result;
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
    clearFpuSet(pingset_ret);

#ifdef FLEXIBLE_CAN_MAPPING
    for (int fpu_id : config.getFpuIdList())
#else
    for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
#endif
    {
        if (fpuset[fpu_id] && (!gs.FPU_state[fpu_id].ping_ok))
        {
            pingset_ret[fpu_id] = true;
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
#ifdef FLEXIBLE_CAN_MAPPING
        for (int fpu_id : config.getFpuIdList())
#else
        for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
#endif
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
        // TODO: The following is adapted from the Python "except" statement,
        // and takes the Python code's exception HIERARCHY into account (as
        // defined in ethercanif.C -> BOOST_PYTHON_MODULE(ethercanif))
        EtherCANErrorGroup error_group = errorGroup(ecan_result);
        if ((error_group == EtherCANErrorGroup::InvalidParameter) ||
            (error_group == EtherCANErrorGroup::Setup) ||
            (error_group == EtherCANErrorGroup::InvalidWaveform) ||
            (error_group == EtherCANErrorGroup::InvalidState) ||
            (error_group == EtherCANErrorGroup::Protection) ||
            (error_group == EtherCANErrorGroup::HardwareProtection))
        {
            // We cancel the datum search altogether, so we can reset positions
            // to old value
            // Note: We do NOT check and return the _cancel_find_datum_hook()
            // return value here, because the startFindDatum() return value above
            // is far more important, and _cancel_find_datum_hook() isn't very 
            // likely to fail anyway
            _cancel_find_datum_hook(gs, fpuset, initial_positions);
        }

        return ecan_result;
    }

    double time_interval = 0.1;
    sleepSecs(time_interval);
    bool is_ready = false;
    bool was_aborted = false;
    bool finished_ok = false;

    abort_motion_pending = false;
    while (!is_ready)
    {
        bool wait_find_datum_finished = false;
        ecan_result = _gd->waitFindDatum(gs, time_interval,
                                         wait_find_datum_finished, &fpuset);

        // Check for abort triggered (N.B. Set from
        // gridDriverAbortDuringFindDatumOrExecuteMotion())
        if (abort_motion_pending)
        {
            abort_motion_pending = false;
            E_EtherCANErrCode abort_result = abortMotion(gs, fpuset);
            if (abort_result != DE_OK)
            {
                ecan_result = abort_result;
            }
            was_aborted = true;
            break;
        }
        is_ready = (ecan_result != DE_WAIT_TIMEOUT);
        finished_ok = (ecan_result == DE_OK);
    }

    if (!finished_ok)
    {
        sleepSecs(time_interval);

        E_EtherCANErrCode ping_if_needed_result = pingIfNeeded(gs, fpuset);
        if (ping_if_needed_result != DE_OK)
        {
            if (errorGroup(ping_if_needed_result) != 
                EtherCANErrorGroup::CommandTimeout)
            {
                return ping_if_needed_result;
            }
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
        return DE_MOVEMENT_ABORTED;
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
#ifdef FLEXIBLE_CAN_MAPPING
    for (int fpu_id : config.getFpuIdList())
#else
    for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
#endif
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

    updateErrorCountersForFpuSet(prev_gs, gs, fpuset);

    if (ecan_result == DE_OK)
    {
        // Clear FPU waveforms
        for (size_t fpu_id = 0; fpu_id < fpus_data.size(); fpu_id++)
        {
            fpus_data[fpu_id].db.last_waveform.clear();
        }
        
        // Wait for FPUs to become active (N.B. values adapted from original
        // Python version of this function)
        sleepSecs(0.1 * 24.0);

        ecan_result = _reset_hook(prev_gs, gs, fpuset);
    }

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

    double alpha_target = (((double)new_alpha_steps) / STEPS_PER_DEGREE_ALPHA) +
                          config.alpha_datum_offset;
    double beta_target = ((double)new_beta_steps) / STEPS_PER_DEGREE_BETA;

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
#ifdef FLEXIBLE_CAN_MAPPING
    for (int fpu_id : config.getFpuIdList())
#else
    for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
#endif
    {
        if (fpuset[fpu_id])
        {
            string_ret += "  FPU[" + std::to_string(fpu_id) + "] ";
        }
    }
    string_ret += "\n";

    string_ret += "  --------   -----------";
#ifdef FLEXIBLE_CAN_MAPPING
    for (int fpu_id : config.getFpuIdList())
#else
    for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
#endif
    {
        if (fpuset[fpu_id])
        {
            string_ret += "  ------ ";
        }
    }
    string_ret += "\n";

    for (size_t i = 0; i < (sizeof(reg_defs) / sizeof(reg_defs[0]));
         i++)
    {
        E_EtherCANErrCode ecan_result = readRegister(reg_defs[i].address,
                                                     gs, fpuset);
        if (ecan_result == DE_OK)   
        {
            string_ret += std::string(reg_defs[i].name) + "        " +
                          std::to_string(reg_defs[i].address);
#ifdef FLEXIBLE_CAN_MAPPING
            for (int fpu_id : config.getFpuIdList())
#else
            for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
#endif
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

    return ecan_result;
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
        // Read FPU's modified serial number back into grid state, and verify
        // that is new serial number
        ecan_result = _gd->readSerialNumbers(gs, fpuset);
    }

    if (ecan_result == DE_OK)
    {
        if (strncmp(gs.FPU_state[fpu_id].serial_number, serial_number,
                    LEN_SERIAL_NUMBER) != 0)
        {
            ecan_result = DE_WRITE_VERIFICATION_FAILED;
        }
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
#ifndef FLEXIBLE_CAN_MAPPING // NOT FLEXIBLE_CAN_MAPPING
            (it->fpu_id < config.num_fpus) &&
#endif // NOT FLEXIBLE_CAN_MAPPING
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

    // TODO: The following is a quite large data structure which will
    // be stored on the local stack - is this OK? (stack overflow?)
    // Or, better to create it on the heap using a std::unique_ptr?
    // (N.B. But it's an array, so if use unique_ptr then would need to
    // encapsulate the array into a structure or class)
    t_grid_state prev_gs;
    _gd->getGridState(prev_gs);

    ecan_result = _gd->configMotion(wtable, gs, fpuset, allow_uninitialized,
                                    ruleset_version);
    EtherCANErrorGroup error_group = errorGroup(ecan_result);
    // N.B. If error group is SocketFailure or CommandTimeout then would be a
    // transmission failure. Here, it is possible that some FPUs have finished
    // loading valid data, but the process was not finished for all FPUs.
    if ((ecan_result == DE_OK) ||
        (error_group == EtherCANErrorGroup::SocketFailure) ||
        (error_group == EtherCANErrorGroup::CommandTimeout))
    {
        // TODO: Much of the following code is inefficient because iterates
        // through arrays/lists many times - but should work OK. This approach
        // is currently taken in order to conform with the existing t_wtable
        // data structure already defined.

        // Accept configured wavetable entries
#ifdef FLEXIBLE_CAN_MAPPING
        for (int fpu_id : config.getFpuIdList())
#else
        for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
#endif
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

        E_EtherCANErrCode post_ecan_result =
                             _post_config_motion_hook(wtable, gs, fpuset);
        // Prioritise any _gd->configMotion() error code first
        if (ecan_result == DE_OK)
        {
            ecan_result = post_ecan_result;
        }
    }

    return ecan_result;
}

//------------------------------------------------------------------------------
void UnprotectedGridDriver::set_wtable_reversed(const t_fpuset &fpuset,
                                                bool is_reversed)
{
    // TODO: Add C++/Linux equivalent of Python version's "with self.lock" here?
    // BUT will also be in UnprotectedGridDriver::_post_config_motion_hook() /
    // _post_repeat_motion_hook() / _post_reverse_motion_hook()? (or at least
    // in the original Python version)

#ifdef FLEXIBLE_CAN_MAPPING
    for (int fpu_id : config.getFpuIdList())
#else
    for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
#endif
    {
        if (fpuset[fpu_id])
        {
            fpus_data[fpu_id].db.wf_reversed = is_reversed;
        }
    }
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
    if (ecan_result != DE_OK)
    {
        EtherCANErrorGroup error_group = errorGroup(ecan_result);
        if ((error_group == EtherCANErrorGroup::InvalidState) ||
            (error_group == EtherCANErrorGroup::Protection))
        {
            // Note: Do NOT want to return the _cancel_execute_motion_hook()
            // return value here, because more interested in the
            // startExecuteMotion() ecan_result value above
            _cancel_execute_motion_hook(gs, fpuset, initial_positions);
        }
        return ecan_result;
    }

    double time_interval = 0.1;
    bool was_aborted = false;
    bool refresh_state = false;

    abort_motion_pending = false;
    while (1)
    {
        bool wait_execute_motion_finished = false;
        ecan_result = _gd->waitExecuteMotion(gs, time_interval,
                                             wait_execute_motion_finished,
                                             fpuset);
        // (N.B. The following check was brought in from 
        // ethercanif.C -> WrapEtherCANInterface::wrap_waitExecuteMotion())
        if ((ecan_result == DE_OK) && (!wait_execute_motion_finished))
        {
            ecan_result = DE_WAIT_TIMEOUT;
        }

        if (ecan_result != DE_WAIT_TIMEOUT)
        {
            break;
        }

        // Check for abort triggered (N.B. Set from
        // gridDriverAbortDuringFindDatumOrExecuteMotion())
        if (abort_motion_pending)
        {
            abort_motion_pending = false;
            ecan_result = abortMotion(gs, fpuset);
            was_aborted = true;
            refresh_state = true;
            break;
        }
    }

    // Check for a movement error (N.B. any one of a number of error groups),
    // or a command timeout.
    // TODO: The movement error groups collated below are based upon the Python
    // version's MovementError exception, which the exception hierarchy in
    // BOOST_PYTHON_MODULE(ethercanif) in ethercanif.C shows has a number of
    // sub-exceptions (see MovementErrorExceptionTypeObj) which correspond to
    // the following error groups - check all of this.
    EtherCANErrorGroup error_group = errorGroup(ecan_result);
    if ((error_group == EtherCANErrorGroup::Collision) ||
        (error_group == EtherCANErrorGroup::LimitBreach) ||
        (error_group == EtherCANErrorGroup::AbortMotion) ||
        (error_group == EtherCANErrorGroup::FirmwareTimeout) ||
        (error_group == EtherCANErrorGroup::Timing) ||
        (error_group == EtherCANErrorGroup::HardwareProtection) ||
        (error_group == EtherCANErrorGroup::CommandTimeout))
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
            if (errorGroup(ping_if_needed_result) != 
                EtherCANErrorGroup::CommandTimeout)
            {
                return ping_if_needed_result;
            }
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

    if (was_aborted)
    {
        return DE_MOVEMENT_ABORTED;
    }

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

    if (ecan_result == DE_OK)
    {
        ecan_result = _post_free_beta_collision_hook(fpu_id, direction, gs);
    }

    return ecan_result;
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

#ifdef FLEXIBLE_CAN_MAPPING
    updateErrorCountersForFpuSet(prev_gs, gs, config.getFpuSet());
#else // NOT FLEXIBLE_CAN_MAPPING
    t_fpuset fpuset;
    createFpuSetForNumFpus(config.num_fpus, fpuset);
    updateErrorCountersForFpuSet(prev_gs, gs, fpuset);
#endif // NOT FLEXIBLE_CAN_MAPPING

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

    if (ecan_result == DE_OK)
    {
        ecan_result = _post_free_alpha_limit_breach_hook(fpu_id, direction, gs);
    }

    return ecan_result;
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

#ifdef FLEXIBLE_CAN_MAPPING
    updateErrorCountersForFpuSet(prev_gs, gs, config.getFpuSet());
#else // NOT FLEXIBLE_CAN_MAPPING
    t_fpuset fpuset;
    createFpuSetForNumFpus(config.num_fpus, fpuset);
    updateErrorCountersForFpuSet(prev_gs, gs, fpuset);
#endif // NOT FLEXIBLE_CAN_MAPPING

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

    if (ecan_result == DE_OK)
    {
        ecan_result = _post_reverse_motion_hook(wtable, gs, fpuset);
    }

    return ecan_result;
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

    if (ecan_result == DE_OK)
    {
        ecan_result = _post_repeat_motion_hook(wtable, gs, fpuset);
    }

    return ecan_result;
}

//------------------------------------------------------------------------------
void UnprotectedGridDriver::buildWtableFromLastWaveforms(const t_fpuset &fpuset,
                                                         t_wtable &wtable_ret)
{
    wtable_ret.clear();
#ifdef FLEXIBLE_CAN_MAPPING
    for (int fpu_id : config.getFpuIdList())
#else
    for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
#endif
    {
        t_waveform_steps &last_waveform = fpus_data[fpu_id].db.last_waveform;
        if (fpuset[fpu_id] && (last_waveform.size() != 0))
        {
            wtable_ret.push_back({(int16_t)fpu_id, last_waveform});
        }
    }
}

//------------------------------------------------------------------------------
void UnprotectedGridDriver::listAngles(const t_grid_state &gs,
                                       t_fpus_angles &fpus_angles_ret,
                                       double alpha_datum_offset,
                                       bool show_uninitialized,
                                       double asteps_per_deg,
                                       double bsteps_per_deg)
{
    // Thin member function wrapper for the separate standalone list_angles()
    // function, to support an easy Boost.Python wrapper function to be created.
    // See the list_angles() comments.
#ifdef FLEXIBLE_CAN_MAPPING
    list_angles(gs, config.getFpuSet(), fpus_angles_ret, alpha_datum_offset,
                show_uninitialized, asteps_per_deg, bsteps_per_deg);
#else // NOT FLEXIBLE_CAN_MAPPING
    list_angles(gs, config.num_fpus, fpus_angles_ret, alpha_datum_offset,
                show_uninitialized, asteps_per_deg, bsteps_per_deg);
#endif // NOT FLEXIBLE_CAN_MAPPING
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
    //
    // Further comments from the grid driver document:
    // The differences to the utility function list_angles() are as follows:
    //   - This function always retrieves the current angles
    //   - This function always uses the same alpha datum offset with which the
    //     driver was configured upon initialisation
    // Therefore, when it is intended to evaluate angles from a previously-
    // retrieved grid_state structure without modifying it, the function
    // list_angles() needs to be used.

    E_EtherCANErrCode ecan_result = checkInitializedAndFpuset(fpuset);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    fpus_angles_ret.clear();

    t_grid_state prev_gs;
    _gd->getGridState(prev_gs);

    ecan_result = _pingFPUs(gs, fpuset);

    updateErrorCountersForFpuSet(prev_gs, gs, fpuset);

    if (ecan_result == DE_OK)
    {
        t_fpus_angles fpus_angles_temp;
#ifdef FLEXIBLE_CAN_MAPPING
        list_angles(gs, fpuset, fpus_angles_temp, config.alpha_datum_offset);
#else // NOT FLEXIBLE_CAN_MAPPING
        list_angles(gs, config.num_fpus, fpus_angles_temp,
                    config.alpha_datum_offset);
#endif // NOT FLEXIBLE_CAN_MAPPING
        for (size_t i = 0; i < fpus_angles_temp.size(); i++)
        {
            if (fpuset[fpus_angles_temp[i].first])
            {
                fpus_angles_ret.push_back(fpus_angles_temp[i]);
            }
        }
    }

    return ecan_result;
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
#ifdef FLEXIBLE_CAN_MAPPING
    for (int fpu_id : config.getFpuIdList())
#else
    for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
#endif
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
#ifdef FLEXIBLE_CAN_MAPPING
    // Checks that all of the FPUs specified in fpuset are part of the list
    // specified in the config.

    const t_fpuset &config_fpuset = config.getFpuSet();
    for (int fpu_id = 0; fpu_id < MAX_NUM_POSITIONERS; fpu_id++)
    {
        if (fpuset[fpu_id] && (!config_fpuset[fpu_id]))
        {
            return DE_INVALID_FPU_ID;
        }
    }

    return DE_OK;

#else // NOT FLEXIBLE_CAN_MAPPING
    // Check that config.num_fpus is within range
    if ((config.num_fpus < 0) || (config.num_fpus >= MAX_NUM_POSITIONERS))
    {
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

#endif // NOT FLEXIBLE_CAN_MAPPING
}

//------------------------------------------------------------------------------
void UnprotectedGridDriver::createFpuSetForSingleFpu(int fpu_id,
                                                     t_fpuset &fpuset_ret)
{
    clearFpuSet(fpuset_ret);
    if ((fpu_id >= 0) && (fpu_id < MAX_NUM_POSITIONERS))
    {
        fpuset_ret[fpu_id] = true;
    }
}

#ifdef FLEXIBLE_CAN_MAPPING
//------------------------------------------------------------------------------
void UnprotectedGridDriver::createFpuSetForIdList(const std::vector<int> &fpu_id_list,
                                                  t_fpuset &fpuset_ret)
{
    clearFpuSet(fpuset_ret);
    for (int fpu_id : fpu_id_list)
    {
        if ((fpu_id >= 0) &&
            (fpu_id < MAX_NUM_POSITIONERS))  // Buffer bounds check
        {
            fpuset_ret[fpu_id] = true;
        }
        else
        {
            // TODO: Error - but should never happen here because fpu_id_list
            // validity should always have been checked elsewhere already
        }
    }
}

#else // NOT FLEXIBLE_CAN_MAPPING
//------------------------------------------------------------------------------
void UnprotectedGridDriver::createFpuSetForNumFpus(int num_fpus,
                                                   t_fpuset &fpuset_ret)
{
    if (num_fpus >= MAX_NUM_POSITIONERS)
    {
        num_fpus = MAX_NUM_POSITIONERS;
    }

    clearFpuSet(fpuset_ret);
    for (int i = 0; i < num_fpus; i++)
    {
        fpuset_ret[i] = true;
    }
}
#endif // NOT FLEXIBLE_CAN_MAPPING

//==============================================================================


} // namespace mpifps

