// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-11-05  Split these functions out from WrappedGridDriver.h.
// bwillemse 2021-03-26  Modified for new non-contiguous FPU IDs and CAN mapping.
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME WrappedGridDriver.C
//
// Boost.Python wrappers for public grid driver functions.
//
////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <signal.h>
#include "WrappedGridDriver.h"
#include "FPUCommands.h"


//==============================================================================
// Abort signal handler functionality
// Supports the actioning of any Ctrl-C abort signal while findDatum() or
// executeMotion() are moving FPUs.
// N.B. The sigaction() function is used instead of signal(), as recommended by
// the signal() manual page https://man7.org/linux/man-pages/man2/signal.2.html. 

static const int abort_handler_signum = SIGINT;

// The following are atomics to make them thread-safe because they are
// modified from the signal handler, which is called from a different thread.
static std::atomic<bool> abort_handler_is_installed(false);
static std::atomic<bool> abort_handler_test_flag(false); // For testing only
// TODO: original_sigaction ideally also needs to be atomic (along with the
// usage of temporary non-atomic structures for the sigaction() calls), but its
// large size isn't easily supported by GCC - see
// https://gcc.gnu.org/bugzilla/show_bug.cgi?id=65756.
static struct sigaction original_sigaction;

static void abortHandlerFunction(int signum);
static void abortHandlerRelease();

//------------------------------------------------------------------------------
void abortHandlerInstall()
{
    // Installs a Linux signal handler for Ctrl-C.

    if (!abort_handler_is_installed)
    {
        struct sigaction abort_sigaction;
        abort_sigaction.sa_handler = abortHandlerFunction;
        sigemptyset(&abort_sigaction.sa_mask);
        abort_sigaction.sa_flags = 0;

        if (sigaction(abort_handler_signum, &abort_sigaction,
                      &original_sigaction) == 0)
        {
            abort_handler_is_installed = true;
        }
    }
}

//------------------------------------------------------------------------------
void abortHandlerUninstall()
{
    // Uninstalls the abort signal handler.

    abortHandlerRelease();
}

//------------------------------------------------------------------------------
void abortHandlerFunction(int signum)
{
    // If the abort handler is installed, then this function is called when
    // Ctrl-C is hit - passes the abort signal to the grid driver instance
    // (if any).

    if (signum == abort_handler_signum)
    {
        abortHandlerRelease();
        gridDriverAbortDuringFindDatumOrExecuteMotion();

        abort_handler_test_flag = true;    // For testing only
    }
}

//------------------------------------------------------------------------------
void abortHandlerRelease()
{
    // Uninstalls the abort signal handler.

    if (abort_handler_is_installed)
    {
        int result = sigaction(abort_handler_signum, &original_sigaction,
                               nullptr);
        abort_handler_is_installed = false;
    }
}

//------------------------------------------------------------------------------
void abortHandlerTest()
{
    // Test function - tests that the abort handler picks up the Ctrl-C signal
    // OK. Can call from e.g. beginning of WrappedGridDriver::initWrapper() for
    // test purposes and do a wrapped grid driver build, then from a Python
    // shell do the following:
    //     from ethercanif import *
    //     gd=GridDriver(1)
    //     ===== Testing Ctrl-C handling =====
    //     Test 1 of 10 - hit Ctrl-C
    //     ^CCtrl-C signal was received OK
    //     Test 2 of 10 - hit Ctrl-C
    //     ^CCtrl-C signal was received OK
    //     ...etc

    std::cout << "===== Testing Ctrl-C handling =====" << std::endl;
    const int num_runs = 10;
    for (int i = 0; i < num_runs; i++)
    {
        std::cout << "Test " << std::to_string(i + 1) << " of " <<
                     std::to_string(num_runs) << " - hit Ctrl-C" << std::endl;
        abortHandlerInstall();
        while (1)
        {
            if (abort_handler_test_flag)
            {
                abort_handler_test_flag = false;
                std::cout << "Ctrl-C signal was received OK" << std::endl;
                break;
            }
        }
        abortHandlerUninstall();
    }    
}


//==============================================================================
boost::shared_ptr<WrappedGridDriver> WrappedGridDriver::initWrapper(
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
    // For Ctrl-C testing only - uncomment to test Ctrl-C detection - see
    // further comments in abortHandlerTest()
    // abortHandlerTest();

#ifndef FLEXIBLE_CAN_MAPPING // NOT FLEXIBLE_CAN_MAPPING
    if ((nfpus <= 0) || (nfpus > MAX_NUM_POSITIONERS))
    {
        std::cout << "*** ERROR ***: nfpus is <=0 or >MAX_NUM_POSITIONERS (" <<
                     std::to_string(MAX_NUM_POSITIONERS) <<
                     ") - GridDriver object created is not valid.\n";
        return boost::shared_ptr<WrappedGridDriver>(nullptr);
    }
#endif // NOT FLEXIBLE_CAN_MAPPING

    std::cout << "Grid driver object was successfully created (new C++ version).\n";

#ifdef FLEXIBLE_CAN_MAPPING
    std::cout << "************************************************************\n";
    std::cout << "************************************************************\n";
    std::cout << "NOTE: The C++ FLEXIBLE_CAN_MAPPING macro is enabled in this\n";
    std::cout << "build, so the new flexible CAN mapping functionality is\n";
    std::cout << "enabled - need to specify the FPUs using a CSV file. The\n";
    std::cout << "old num_fpus approach is not supported in this build.\n";
    std::cout << "************************************************************\n";
    std::cout << "************************************************************\n";
    std::cout << std::endl;
#endif // FLEXIBLE_CAN_MAPPING

#ifndef ENABLE_PROTECTION_CODE  // NOT ENABLE_PROTECTION_CODE
    std::cout << "************************************************************\n";
    std::cout << "************************************************************\n";
    std::cout << "NOTE: The C++ ENABLE_PROTECTION_CODE macro is disabled in\n";
    std::cout << "this build, so the soft protection is not functional.\n";
    std::cout << "************************************************************\n";
    std::cout << "************************************************************\n";
    std::cout << std::endl;
#endif // NOT ENABLE_PROTECTION_CODE

    if (confirm_each_step)
    {
        std::cout << "\nconfirm_each_step is set to True, which requires extra confirmation\n";
        std::cout << "requests of waveform step upload, and reduces performance\n" << std::endl;
    }

    if (min_bus_repeat_delay_ms > 0)
    {
        std::cout << "\nmin_bus_repeat_delay_ms is set to value above 0.\n";
        std::cout << "Decrease if message rate is too low.\n" << std::endl;
    }

    return boost::shared_ptr<WrappedGridDriver>(new WrappedGridDriver(
#ifndef FLEXIBLE_CAN_MAPPING // NOT FLEXIBLE_CAN_MAPPING
        nfpus,
#endif // NOT FLEXIBLE_CAN_MAPPING
        SocketTimeOutSeconds,
        confirm_each_step,
        waveform_upload_pause_us,
        configmotion_max_retry_count,
        configmotion_max_resend_count,
        min_bus_repeat_delay_ms,
        min_fpu_repeat_delay_ms,
        alpha_datum_offset,
        motor_minimum_frequency,
        motor_maximum_frequency,
        motor_max_start_frequency,
        motor_max_rel_increase,
        motor_max_step_difference));
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_initialize(
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
                                        const std::string &start_timestamp,
                                        bool mockup)
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (!initializedOk())   // Only initialise if not already done
    {
#ifdef FLEXIBLE_CAN_MAPPING
        ecode = initialize(can_map_file_path, logLevel, log_dir,
                           firmware_version_address_offset, protection_logfile,
                           control_logfile, tx_logfile, rx_logfile,
                           start_timestamp);
#else // NOT FLEXIBLE_CAN_MAPPING
        ecode = initialize(logLevel, log_dir, firmware_version_address_offset,
                           protection_logfile, control_logfile, tx_logfile,
                           rx_logfile, start_timestamp);
#endif // NOT FLEXIBLE_CAN_MAPPING
        if (ecode == DE_OK)
        {
            ecode = initProtection(mockup);
            if (ecode != DE_OK)
            {
                std::cout << "*** ERROR ***: initProtection() call failed "
                             "during the initialize command" << std::endl;
            }
        }
        else
        {
            std::cout << "*** ERROR ***: initialize() call failed during "
                         "the initialize command" << std::endl;
        }
    }
    else
    {
        std::cout << "Warning: initialize() has already been called - skipping."
                  << std::endl;
        ecode = DE_OK;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
WrapGridState WrappedGridDriver::wrapped_getGridState()
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;
    
    WrapGridState grid_state;
    if (checkAndMessageIfInitializedOk())
    {
        /*E_GridState grid_state_enum = */ getGridState(grid_state);
        ecode = DE_OK;
    }
    else
    {
        // TODO: Zero grid_state here, using e.g. memset? BUT WrapGridState
        // is a class rather than a POD structure, so would this be OK?

        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);

    return grid_state;  // N.B. Returning grid_state rather than ecode
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_connect(bp::list &list_gateway_addresses)
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (checkAndMessageIfInitializedOk())
    {
        t_gateway_address address_array[MAX_NUM_GATEWAYS];
        const int actual_num_gw = convertGatewayAddresses(list_gateway_addresses,
                                                          address_array);
        ecode = connect(actual_num_gw, address_array);
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_disconnect()
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (checkAndMessageIfInitializedOk())
    {
        ecode = disconnect();
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_setUStepLevel(int ustep_level,
                                                           WrapGridState &grid_state,
                                                           bp::list &fpu_list)
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (checkAndMessageIfInitializedOk())
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        ecode = setUStepLevel(ustep_level, grid_state, fpuset);
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_setTicksPerSegment(
                                                    unsigned long ticks,
                                                    WrapGridState &grid_state,
                                                    bp::list &fpu_list)
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (checkAndMessageIfInitializedOk())
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        ecode = setTicksPerSegment(ticks, grid_state, fpuset);
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_setStepsPerSegment(int min_steps,
                                                                int max_steps,
                                                    WrapGridState &grid_state,
                                                    bp::list &fpu_list)
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (checkAndMessageIfInitializedOk())
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        ecode = setStepsPerSegment(min_steps, max_steps, grid_state, fpuset);
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_findDatum(WrapGridState &grid_state,
                                                bp::dict &dict_search_modes,
                                                E_DATUM_SELECTION selected_arm,
                                                bp::list &fpu_list,
                                                bool soft_protection,
                                                bool count_protection,
                                                bool support_uninitialized_auto,
                                                E_DATUM_TIMEOUT_FLAG timeout)
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (checkAndMessageIfInitializedOk())
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        t_datum_search_flags direction_flags;
        getDatumFlags(dict_search_modes, direction_flags, fpuset);

        abortHandlerInstall();   // Provides Ctrl-C aborting during FPU motion
        ecode = findDatum(grid_state, direction_flags, selected_arm, fpuset,
                          soft_protection, count_protection, 
                          support_uninitialized_auto, timeout);
        abortHandlerUninstall();
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_resetFPUs(WrapGridState& grid_state,
                                                       list& fpu_list)
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (checkAndMessageIfInitializedOk())
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        ecode = resetFPUs(grid_state, fpuset);
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_resetStepCounters(long new_alpha_steps,
                                                               long new_beta_steps,
                                                               WrapGridState &grid_state,
                                                               bp::list &fpu_list)
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (checkAndMessageIfInitializedOk())
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        ecode = resetStepCounters(new_alpha_steps, new_beta_steps, grid_state,
                                  fpuset);
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_readRegister(int read_address,
                                                          WrapGridState &grid_state,
                                                          bp::list &fpu_list)
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (checkAndMessageIfInitializedOk())
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        if ((read_address >= 0) && (read_address <= 0xffff))
        {
            const uint16_t raddress = (uint16_t)read_address;
            ecode = readRegister(raddress, grid_state, fpuset);
        }
        else
        {
            ecode = DE_INVALID_PAR_VALUE;
        }
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_getDiagnostics(WrapGridState &grid_state,
                                                            bp::list &fpu_list)
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;
    std::string diag_string;

    if (checkAndMessageIfInitializedOk())
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        ecode = getDiagnostics(grid_state, fpuset, diag_string);
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);

    std::cout << diag_string << std::endl;

    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_pingFPUs(WrapGridState &grid_state,
                                                      bp::list &fpu_list)
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (checkAndMessageIfInitializedOk())
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        ecode = pingFPUs(grid_state, fpuset);
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_getFirmwareVersion(
                                                    WrapGridState &grid_state,
                                                    bp::list & fpu_list)
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (checkAndMessageIfInitializedOk())
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        ecode = getFirmwareVersion(grid_state, fpuset);
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_readSerialNumbers(WrapGridState &grid_state,
                                                               bp::list &fpu_list)
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (checkAndMessageIfInitializedOk())
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        ecode = readSerialNumbers(grid_state, fpuset);
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_writeSerialNumber(int fpu_id,
                                                               bp::str serial_number,
                                                               WrapGridState &grid_state)
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (checkAndMessageIfInitializedOk())
    {
        std::string cpp_serial_number = bp::extract<std::string>(serial_number);

        ecode = writeSerialNumber(fpu_id, cpp_serial_number.c_str(), grid_state);
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_configMotion(bp::dict &dict_waveforms,
                                                    WrapGridState &grid_state,
                                                    bp::list &fpu_list,
                                                    bool soft_protection,
                                                    bool allow_uninitialized,
                                                    int ruleset_version,
                                                    // TODO: The following arguments
                                                    // are from FpuGridDriver.py ->
                                                    // configMotion() - keep?
                                                    bool warn_unsafe, int verbosity)
{
    // Configures movement by sending a waveform table to a group of FPUs.
    // Call signature is:
    // configMotion( { fpuid0 : { (asteps, bsteps), (asteps, bsteps), ...],
    //                 fpuid1 : { ... }, ...}})

    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (checkAndMessageIfInitializedOk())
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        t_wtable wtable;
        convertWavetable(dict_waveforms, wtable);

        ecode = configMotion(wtable, grid_state, fpuset, soft_protection,
                             allow_uninitialized, ruleset_version, warn_unsafe,
                             verbosity);
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_executeMotion(WrapGridState &grid_state,
                                                           bp::list &fpu_list,
                                                           bool sync_command)
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (checkAndMessageIfInitializedOk())
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        abortHandlerInstall();   // Provides Ctrl-C aborting during FPU motion
        ecode = executeMotion(grid_state, fpuset, sync_command);
        abortHandlerUninstall();
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_abortMotion(WrapGridState &grid_state,
                                                         bp::list &fpu_list,
                                                         bool sync_command)
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (checkAndMessageIfInitializedOk())
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        ecode = abortMotion(grid_state, fpuset, sync_command);
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_freeBetaCollision(int fpu_id,
                                                E_REQUEST_DIRECTION direction,
                                                WrapGridState &grid_state,
                                                bool soft_protection)
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (checkAndMessageIfInitializedOk())
    {
        ecode = freeBetaCollision(fpu_id, direction, grid_state,
                                  soft_protection);
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_enableBetaCollisionProtection(
                                            WrapGridState &grid_state)
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (checkAndMessageIfInitializedOk())
    {
        ecode = enableBetaCollisionProtection(grid_state);
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_freeAlphaLimitBreach(int fpu_id,
                                                E_REQUEST_DIRECTION direction,
                                                WrapGridState &grid_state,
                                                bool soft_protection)
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (checkAndMessageIfInitializedOk())
    {
        ecode = freeAlphaLimitBreach(fpu_id, direction, grid_state,
                                     soft_protection);
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_enableAlphaLimitProtection(
                                                    WrapGridState &grid_state)
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (checkAndMessageIfInitializedOk())
    {
        ecode = enableAlphaLimitProtection(grid_state);
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_reverseMotion(WrapGridState &grid_state,
                                                           bp::list &fpu_list,
                                                           bool soft_protection)
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (checkAndMessageIfInitializedOk())
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        ecode = reverseMotion(grid_state, fpuset, soft_protection);
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_repeatMotion(WrapGridState &grid_state,
                                                          bp::list &fpu_list,
                                                          bool soft_protection)
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (checkAndMessageIfInitializedOk())
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        ecode = repeatMotion(grid_state, fpuset, soft_protection);
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_lockFPU(int fpu_id, 
                                                     WrapGridState &grid_state)
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (checkAndMessageIfInitializedOk())
    {
        ecode = lockFPU(fpu_id, grid_state);
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_unlockFPU(int fpu_id,
                                                       WrapGridState &grid_state)
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (checkAndMessageIfInitializedOk())
    {
        ecode = unlockFPU(fpu_id, grid_state);
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_enableMove(int fpu_id,
                                                        WrapGridState &grid_state)
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (checkAndMessageIfInitializedOk())
    {
        ecode = enableMove(fpu_id, grid_state);
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_checkIntegrity(WrapGridState &grid_state,
                                                            bp::list &fpu_list)
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (checkAndMessageIfInitializedOk())
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        ecode = checkIntegrity(grid_state, fpuset);
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_list_angles(WrapGridState &grid_state,
                                                         double alpha_datum_offset,
                                                         bool show_uninitialized,
                                                         double asteps_per_deg,
                                                         double bsteps_per_deg)
{
    // TODO: This wrapper currently only produces a string which is output
    // to the Python console - but the native Python list_angles() returns
    // a data structure - so need to mimic that eventually?

    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (checkAndMessageIfInitializedOk())
    {
        t_fpus_angles fpus_angles;
        listAngles(grid_state, fpus_angles, alpha_datum_offset, show_uninitialized,
                   asteps_per_deg, bsteps_per_deg);

        std::string angles_string;
        createFpuDoublesAnglesString(fpus_angles, angles_string);
        std::cout << angles_string << std::endl;
        ecode = DE_OK;
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_countedAngles(WrapGridState &grid_state,
                                                           bp::list &fpu_list,
                                                           bool show_uninitialized)
{
    // TODO: This wrapper currently only produces a string which is output
    // to the Python console - but the native Python list_angles() returns
    // a data structure - so need to mimic that eventually?

    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

    if (checkAndMessageIfInitializedOk())
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        t_fpus_angles fpus_angles;
        ecode = countedAngles(grid_state, fpuset, fpus_angles,
                              show_uninitialized);
        if (ecode == DE_OK)
        {
            std::string angles_string;
            createFpuDoublesAnglesString(fpus_angles, angles_string);
            std::cout << angles_string << std::endl;
        }
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
void WrappedGridDriver::createFpuDoublesAnglesString(const t_fpus_angles &fpus_angles,
                                                     std::string &angles_string_ret)
{
    angles_string_ret.clear();
    for (const auto &it : fpus_angles)
    {
        angles_string_ret += "FPU " + std::to_string(it.first) + 
                             ": angles = (" +
                             doubleToString(it.second.alpha) + ", " +
                             doubleToString(it.second.beta) + ")\n";
    }
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_trackedAngles(WrapGridState &grid_state,
                                                           bp::list &fpu_list,
                                                           bool show_offsets,
                                                           bool active)
{
    E_EtherCANErrCode ecode = DE_ERROR_UNKNOWN;

#ifdef ENABLE_PROTECTION_CODE
    if (checkAndMessageIfInitializedOk())
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        std::string angles_string;
        ecode = trackedAnglesString(grid_state, fpuset, angles_string,
                                    show_offsets, active);
        if (ecode == DE_OK)
        {
            std::cout << angles_string << std::endl;
        }
    }
    else
    {
        ecode = DE_INTERFACE_NOT_INITIALIZED;
    }

#else // NOT ENABLE_PROTECTION_CODE
    std::cout << "************************************************************\n";
    std::cout << "NOTE: The C++ ENABLE_PROTECTION_CODE macro is disabled in\n";
    std::cout << "this build, so trackedAngles() is not available.\n";
    std::cout << "************************************************************\n";
    std::cout << std::endl;
    ecode = DE_FIRMWARE_UNIMPLEMENTED;
#endif // NOT ENABLE_PROTECTION_CODE

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
bool WrappedGridDriver::checkAndMessageIfInitializedOk()
{
    if (initializedOk())
    {
        return true;
    }
    std::cout << std::endl;
    std::cout << "*** ERROR ***: Not yet initialized successfully - "
                 "initialize() was not yet called, or it failed" << std::endl;
    return false;
}

//------------------------------------------------------------------------------
const EtherCANInterfaceConfig &WrappedGridDriver::getConfig() const
{
    return config;
}

//==============================================================================

