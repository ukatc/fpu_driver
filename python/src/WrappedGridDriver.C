// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-11-05  Split these functions out from WrappedGridDriver.h.
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


//==============================================================================
// Abort signal handler functionality
// Supports the actioning of any Ctrl-C abort signal while findDatum() or
// executeMotion() are moving FPUs.

// USE_SIGACTION: Specifies to use sigaction() function instead of signal(),
// as recommended by the signal() manual page comments.
#define USE_SIGACTION

static const int abort_handler_signum = SIGINT;

// The following are atomics to make them thread-safe, because they are
// modified from the signal handler, which is called from a different thread.
// TODO: Would these variables be at risk of the C++ static initialization
// order problem / fiasco?   
static std::atomic<bool> abort_handler_is_installed(false);

#ifdef USE_SIGACTION
// ********* TODO: Ideally make original_sigaction atomic
static struct sigaction original_sigaction;
#else  // NOT USE_SIGACTION
static std::atomic<sighandler_t> signal_handler_original(nullptr);
#endif  // NOT USE_SIGACTION

static std::atomic<bool> abortHandlerTestFlag(false); // For testing only

static void abortHandlerFunction(int signum);
static void abortHandlerRelease();

//------------------------------------------------------------------------------
void abortHandlerInstall()
{
    // Installs a Linux signal handler for Ctrl-C.

    if (!abort_handler_is_installed)
    {
#ifdef USE_SIGACTION
        struct sigaction abort_sigaction;
        abort_sigaction.sa_handler = abortHandlerFunction,
        sigemptyset(&abort_sigaction.sa_mask);
        abort_sigaction.sa_flags = 0;

        int result = sigaction(abort_handler_signum, &abort_sigaction,
                               &original_sigaction);
#else  // NOT USE_SIGACTION
        signal_handler_original = signal(abort_handler_signum,
                                         abortHandlerFunction);
#endif  // NOT USE_SIGACTION
        abort_handler_is_installed = true;
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

        abortHandlerTestFlag = true;    // For testing only
    }
}

//------------------------------------------------------------------------------
void abortHandlerRelease()
{
    // Uninstalls the abort signal handler.

    if (abort_handler_is_installed)
    {
#ifdef USE_SIGACTION
        int result = sigaction(abort_handler_signum, &original_sigaction,
                               nullptr);
#else  // NOT USE_SIGACTION
        signal(abort_handler_signum, signal_handler_original);
#endif  // NOT USE_SIGACTION
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
            if (abortHandlerTestFlag)
            {
                abortHandlerTestFlag = false;
                std::cout << "Ctrl-C signal was received OK" << std::endl;
                break;
            }
        }
        abortHandlerUninstall();
    }    
}


//==============================================================================
boost::shared_ptr<WrappedGridDriver> WrappedGridDriver::initWrapper(
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
    // For Ctrl-C testing only - uncomment to test Ctrl-C detection - see
    // further comments in abortHandlerTest()
    // abortHandlerTest();

    if ((nfpus <= 0) || (nfpus > MAX_NUM_POSITIONERS))
    {
        std::cout << "*** ERROR ***: nfpus is <=0 or >MAX_NUM_POSITIONERS (" <<
                     std::to_string(MAX_NUM_POSITIONERS) <<
                     ") - GridDriver object created is not valid.\n";
        return boost::shared_ptr<WrappedGridDriver>(nullptr);
    }

    std::cout << "Grid driver object was successfully created (new C++ version).\n";

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
        nfpus,
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
E_EtherCANErrCode WrappedGridDriver::wrapped_initialize(E_LogLevel logLevel,
                                        const std::string &log_dir,
                                        int firmware_version_address_offset,
                                        const std::string &protection_logfile,
                                        const std::string &control_logfile,
                                        const std::string &tx_logfile,
                                        const std::string &rx_logfile,
                                        const std::string &start_timestamp,
                                        bool mockup)
{
    if (initializedOk())
    {
        return DE_INTERFACE_ALREADY_INITIALIZED;
    }

    E_EtherCANErrCode ecode = initialize(logLevel, log_dir,
                                         firmware_version_address_offset,
                                         protection_logfile,
                                         control_logfile,
                                         tx_logfile, rx_logfile,
                                         start_timestamp);
    if ((ecode == DE_OK) || (ecode == DE_INTERFACE_ALREADY_INITIALIZED))
    {
        ecode = initProtection(mockup);
        if ((ecode != DE_OK) && (ecode != DE_INTERFACE_ALREADY_INITIALIZED))
        {
            std::cout << "*** ERROR ***: Protection initialisation failed" << std::endl;
        }
    }

    return ecode;
}

//------------------------------------------------------------------------------
WrapGridState WrappedGridDriver::wrapped_getGridState()
{
    WrapGridState grid_state;
    if (checkAndMessageIfInitializedOk())
    {
        /*E_GridState grid_state_enum = */ getGridState(grid_state);
    }
    else
    {
        // TODO: Zero grid_state here, using e.g. memset? BUT WrapGridState
        // is a class rather than a POD structure, so would this be OK?
    }
    return grid_state;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_connect(bp::list &list_gateway_addresses)
{
    if (!checkAndMessageIfInitializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    t_gateway_address address_array[MAX_NUM_GATEWAYS];
    const int actual_num_gw = convertGatewayAddresses(list_gateway_addresses,
                                                      address_array);
    E_EtherCANErrCode ecode = connect(actual_num_gw, address_array);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_disconnect()
{
    if (!checkAndMessageIfInitializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    E_EtherCANErrCode ecode = disconnect();
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_setUStepLevel(int ustep_level,
                                                           WrapGridState &grid_state,
                                                           bp::list &fpu_list)
{
    if (!checkAndMessageIfInitializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    E_EtherCANErrCode ecode = setUStepLevel(ustep_level, grid_state, fpuset);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_setTicksPerSegment(
                                                    unsigned long ticks,
                                                    WrapGridState &grid_state,
                                                    bp::list &fpu_list)
{
    if (!checkAndMessageIfInitializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    E_EtherCANErrCode ecode = setTicksPerSegment(ticks, grid_state, fpuset);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_setStepsPerSegment(int min_steps,
                                                                int max_steps,
                                                    WrapGridState &grid_state,
                                                    bp::list &fpu_list)
{
    if (!checkAndMessageIfInitializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    E_EtherCANErrCode ecode = setStepsPerSegment(min_steps, max_steps,
                                                 grid_state, fpuset);
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
    if (!checkAndMessageIfInitializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    t_datum_search_flags direction_flags;
    getDatumFlags(dict_search_modes, direction_flags, fpuset);

    abortHandlerInstall();   // Provides Ctrl-C aborting during FPU motion
    E_EtherCANErrCode ecode = findDatum(grid_state, direction_flags,
                                        selected_arm, fpuset, soft_protection,
                                        count_protection, 
                                        support_uninitialized_auto, timeout);
    abortHandlerUninstall();

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_resetFPUs(WrapGridState& grid_state,
                                                       list& fpu_list)
{
    if (!checkAndMessageIfInitializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    E_EtherCANErrCode ecode = resetFPUs(grid_state, fpuset);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_resetStepCounters(long new_alpha_steps,
                                                               long new_beta_steps,
                                                               WrapGridState &grid_state,
                                                               bp::list &fpu_list)
{
    if (!checkAndMessageIfInitializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    E_EtherCANErrCode ecode = resetStepCounters(new_alpha_steps, new_beta_steps,
                                                grid_state, fpuset);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_readRegister(int read_address,
                                                          WrapGridState &grid_state,
                                                          bp::list &fpu_list)
{
    if (!checkAndMessageIfInitializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    if ((read_address > 0xffff) || (read_address < 0))
    {
        checkInterfaceError(DE_INVALID_PAR_VALUE);
    }
    const uint16_t raddress = (uint16_t)read_address;
    E_EtherCANErrCode ecode = readRegister(raddress, grid_state, fpuset);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_getDiagnostics(WrapGridState &grid_state,
                                                            bp::list &fpu_list)
{
    if (!checkAndMessageIfInitializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    std::string diag_string;
    E_EtherCANErrCode ecode = getDiagnostics(grid_state, fpuset, diag_string);

    checkInterfaceError(ecode);

    std::cout << diag_string << std::endl;

    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_pingFPUs(WrapGridState &grid_state,
                                                      bp::list &fpu_list)
{
    if (!checkAndMessageIfInitializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    E_EtherCANErrCode ecode = pingFPUs(grid_state, fpuset);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_getFirmwareVersion(
                                                    WrapGridState &grid_state,
                                                    bp::list & fpu_list)
{
    if (!checkAndMessageIfInitializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    E_EtherCANErrCode ecode = getFirmwareVersion(grid_state, fpuset);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_readSerialNumbers(WrapGridState &grid_state,
                                                               bp::list &fpu_list)
{
    if (!checkAndMessageIfInitializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    E_EtherCANErrCode ecode = readSerialNumbers(grid_state, fpuset);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_writeSerialNumber(int fpu_id,
                                                               bp::str serial_number,
                                                               WrapGridState &grid_state)
{
    if (!checkAndMessageIfInitializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    std::string cpp_serial_number = bp::extract<std::string>(serial_number);

    E_EtherCANErrCode ecode = writeSerialNumber(fpu_id,
                                                cpp_serial_number.c_str(),
                                                grid_state);
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
    if (!checkAndMessageIfInitializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    t_wtable wtable;
    convertWavetable(dict_waveforms, wtable);

    E_EtherCANErrCode ecode = configMotion(wtable, grid_state, fpuset,
                                           soft_protection, allow_uninitialized,
                                           ruleset_version, warn_unsafe,
                                           verbosity);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_executeMotion(WrapGridState &grid_state,
                                                           bp::list &fpu_list,
                                                           bool sync_command)
{
    if (!checkAndMessageIfInitializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    abortHandlerInstall();   // Provides Ctrl-C aborting during FPU motion
    E_EtherCANErrCode ecode = executeMotion(grid_state, fpuset, sync_command);
    abortHandlerUninstall();

    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_abortMotion(WrapGridState &grid_state,
                                                         bp::list &fpu_list,
                                                         bool sync_command)
{
    if (!checkAndMessageIfInitializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    E_EtherCANErrCode ecode = abortMotion(grid_state, fpuset, sync_command);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_freeBetaCollision(int fpu_id,
                                                E_REQUEST_DIRECTION direction,
                                                WrapGridState &grid_state,
                                                bool soft_protection)
{
    if (!checkAndMessageIfInitializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    E_EtherCANErrCode ecode = freeBetaCollision(fpu_id, direction, grid_state,
                                                soft_protection);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_enableBetaCollisionProtection(
                                            WrapGridState &grid_state)
{
    if (!checkAndMessageIfInitializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    E_EtherCANErrCode ecode = enableBetaCollisionProtection(grid_state);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_freeAlphaLimitBreach(int fpu_id,
                                                E_REQUEST_DIRECTION direction,
                                                WrapGridState &grid_state,
                                                bool soft_protection)
{
    if (!checkAndMessageIfInitializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    E_EtherCANErrCode ecode = freeAlphaLimitBreach(fpu_id, direction,
                                                   grid_state, soft_protection);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_enableAlphaLimitProtection(
                                                    WrapGridState &grid_state)
{
    if (!checkAndMessageIfInitializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    E_EtherCANErrCode ecode = enableAlphaLimitProtection(grid_state);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_reverseMotion(WrapGridState &grid_state,
                                                           bp::list &fpu_list,
                                                           bool soft_protection)
{
    if (!checkAndMessageIfInitializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    E_EtherCANErrCode ecode = reverseMotion(grid_state, fpuset, soft_protection);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_repeatMotion(WrapGridState &grid_state,
                                                          bp::list &fpu_list,
                                                          bool soft_protection)
{
    if (!checkAndMessageIfInitializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    E_EtherCANErrCode ecode = repeatMotion(grid_state, fpuset, soft_protection);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_lockFPU(int fpu_id, 
                                                     WrapGridState &grid_state)
{
    if (!checkAndMessageIfInitializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    E_EtherCANErrCode ecode = lockFPU(fpu_id, grid_state);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_unlockFPU(int fpu_id,
                                                       WrapGridState &grid_state)
{
    if (!checkAndMessageIfInitializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    E_EtherCANErrCode ecode = unlockFPU(fpu_id, grid_state);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_enableMove(int fpu_id,
                                                        WrapGridState &grid_state)
{
    if (!checkAndMessageIfInitializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    E_EtherCANErrCode ecode = enableMove(fpu_id, grid_state);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_checkIntegrity(WrapGridState &grid_state,
                                                            bp::list &fpu_list)
{
    if (!checkAndMessageIfInitializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    E_EtherCANErrCode ecode = checkIntegrity(grid_state, fpuset);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrappedGridDriver::wrapped_trackedAngles(WrapGridState &grid_state,
                                                           bp::list &fpu_list,
                                                           bool show_offsets,
                                                           bool active)
{
#ifdef ENABLE_PROTECTION_CODE
    if (!checkAndMessageIfInitializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    std::string angles_string;
    E_EtherCANErrCode ecode = trackedAngles(grid_state, fpuset, angles_string,
                                            show_offsets, active);
    checkInterfaceError(ecode);

    std::cout << angles_string << std::endl;

#else // NOT ENABLE_PROTECTION_CODE
    std::cout << "************************************************************\n";
    std::cout << "NOTE: The C++ ENABLE_PROTECTION_CODE macro is disabled in\n";
    std::cout << "this build, so trackedAngles() is not available.\n";
    std::cout << "************************************************************\n";
    std::cout << std::endl;
    E_EtherCANErrCode ecode = DE_FIRMWARE_UNIMPLEMENTED;
#endif // NOT ENABLE_PROTECTION_CODE

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
    std::cout << "*** ERROR ***: Not yet initialized successfully - initialize() was not yet called, or it failed\n" << std::endl;
    return false;
}

//------------------------------------------------------------------------------
const EtherCANInterfaceConfig &WrappedGridDriver::getConfig() const
{
    return config;
}

//==============================================================================

