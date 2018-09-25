// -*- mode: c++ -*-
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 UKRI. See file "LICENSE" for license information.
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client sample
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME AsyncDriver.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////
#include <cassert>
#include <unistd.h>
#include <string.h>

#ifdef DEBUG
#include <stdio.h>
#endif

#include "canlayer/AsyncDriver.h"
#include "canlayer/GatewayDriver.h"
#include "canlayer/time_utils.h"

// we need to include the individual CAN commands
// as they are parametrized here.
// -- alphabetically sorted below --
#include "canlayer/commands/AbortMotionCommand.h"
#include "canlayer/commands/ConfigureMotionCommand.h"
#include "canlayer/commands/EnableBetaCollisionProtectionCommand.h"
#include "canlayer/commands/ExecuteMotionCommand.h"
#include "canlayer/commands/FindDatumCommand.h"
#include "canlayer/commands/FreeBetaCollisionCommand.h"
#include "canlayer/commands/GetErrorAlphaCommand.h"
#include "canlayer/commands/GetErrorBetaCommand.h"
#include "canlayer/commands/GetStepsAlphaCommand.h"
#include "canlayer/commands/GetStepsBetaCommand.h"
#include "canlayer/commands/PingFPUCommand.h"
#include "canlayer/commands/ReadRegisterCommand.h"
#include "canlayer/commands/ReadSerialNumberCommand.h"
#include "canlayer/commands/RepeatMotionCommand.h"
#include "canlayer/commands/ResetFPUCommand.h"
#include "canlayer/commands/ReverseMotionCommand.h"
#include "canlayer/commands/SetUStepLevelCommand.h"
#include "canlayer/commands/WriteSerialNumberCommand.h"


namespace mpifps
{

namespace canlayer
{

E_DriverErrCode AsyncDriver::initializeDriver()
{
    switch (gateway.getDriverState())
    {
    case DS_UNINITIALIZED:
        break;

    case DS_UNCONNECTED:
    case DS_CONNECTED:
        LOG_CONTROL(LOG_ERROR, "%18.6f : initializeDriver() - driver was already initialized\n",
                    canlayer::get_realtime());
        return DE_DRIVER_ALREADY_INITIALIZED;

    case DS_ASSERTION_FAILED:
    default:
        LOG_CONTROL(LOG_ERROR, "%18.6f : error during initializeDriver() - assertion failed\n",
                    canlayer::get_realtime());
        return DE_ASSERTION_FAILED;
    }
    LOG_CONTROL(LOG_DEBUG, "%18.6f : initializing driver\n",
                canlayer::get_realtime());
    return gateway.initialize();
}


E_DriverErrCode AsyncDriver::deInitializeDriver()
{
    switch (gateway.getDriverState())
    {
    case DS_ASSERTION_FAILED:
    case DS_UNCONNECTED:
        break;

    case DS_UNINITIALIZED:
        LOG_CONTROL(LOG_ERROR, "%18.6f : deinitializeDriver() - error: driver is already in uninitialized state \n",
                    canlayer::get_realtime());
        return DE_DRIVER_NOT_INITIALIZED;

    case DS_CONNECTED:
        LOG_CONTROL(LOG_ERROR, "%18.6f : deinitializeDriver() - error: can't deinitialize driver, it is still connected\n",
                    canlayer::get_realtime());
        return DE_DRIVER_STILL_CONNECTED;

    default:
        LOG_CONTROL(LOG_ERROR, "%18.6f : deinitializeDriver() - fatal error: assertion failed\n",
                    canlayer::get_realtime());
        return DE_ASSERTION_FAILED;
    };

    LOG_CONTROL(LOG_INFO, "%18.6f : deinitializing driver\n",
                canlayer::get_realtime());
    return gateway.deInitialize();
}

E_DriverErrCode AsyncDriver::connect(const int ngateways, const t_gateway_address gateway_addresses[])
{
    switch (gateway.getDriverState())
    {
    case DS_UNINITIALIZED:
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncDriver::connect(): error: driver not initialized\n",
                    canlayer::get_realtime());
        return DE_DRIVER_NOT_INITIALIZED;

    case DS_UNCONNECTED:
        break;

    case DS_CONNECTED:
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncDriver::connect(): error: driver already connected, needs to disconnect() first\n",
                    canlayer::get_realtime());
        return DE_DRIVER_ALREADY_CONNECTED;

    case DS_ASSERTION_FAILED:
    default:
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncDriver::connect(): fatal error: assertion failed\n",
                    canlayer::get_realtime());
        return DE_ASSERTION_FAILED;
    }

    // Make sure that the passed number of gateways can support the
    // configured number of FPUs.
    if (ngateways < (config.num_fpus + MAX_FPUS_PER_GATEWAY-1) / MAX_FPUS_PER_GATEWAY)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncDriver::connect(): number of configured gateways is insufficient\n",
                    canlayer::get_realtime());
        return DE_INSUFFICENT_NUM_GATEWAYS;
    }

    E_DriverErrCode err_code =  gateway.connect(ngateways, gateway_addresses);
    if (err_code == DE_OK)
    {
        num_gateways = ngateways;
    }
    LOG_CONTROL(LOG_INFO, "%18.6f : GridDriver::connect(): driver is connected to %i gateways\n",
                canlayer::get_realtime(),
                num_gateways);

    return err_code;
}

E_DriverErrCode AsyncDriver::disconnect()
{

    switch (gateway.getDriverState())
    {
    case DS_UNINITIALIZED:
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncDriver::disconnect(): error, driver not initialized\n",
                    canlayer::get_realtime());
        return DE_DRIVER_NOT_INITIALIZED;

    case DS_UNCONNECTED:
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncDriver::disconnect(): error, driver not connected\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;

    case DS_CONNECTED:
    case DS_ASSERTION_FAILED:
    default:
        break;
    }

    LOG_CONTROL(LOG_DEBUG, "%18.6f : AsyncDriver::disconnect(): disconnecting driver\n",
                canlayer::get_realtime());


    E_DriverErrCode err_code = gateway.disconnect();

    if (err_code == DE_OK)
    {
        num_gateways = 0;
        LOG_CONTROL(LOG_DEBUG, "%18.6f : disconnect(): OK\n",
                    canlayer::get_realtime());

    }

    return err_code;

}


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

E_DriverErrCode AsyncDriver::initializeGridAsync(t_grid_state& grid_state,
        E_GridState& state_summary,
        t_fpuset const &fpuset)
{

    state_summary = GS_UNKNOWN;
    grid_state.driver_state = DS_ASSERTION_FAILED;

    LOG_CONTROL(LOG_INFO, "%18.6f : initializing grid\n",
                canlayer::get_realtime());

    if (gateway.getDriverState() != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : initializeGridAsync() error: driver is not connected\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }

    LOG_CONTROL(LOG_INFO, "%18.6f : initializeGrid(): command successfully sent\n",
                canlayer::get_realtime());

    return DE_OK;
}
#pragma GCC diagnostic pop


E_DriverErrCode AsyncDriver::resetFPUsAsync(t_grid_state& grid_state,
        E_GridState& state_summary,
        t_fpuset const &fpuset)
{

    LOG_CONTROL(LOG_INFO, "%18.6f : resetting FPUs\n",
                canlayer::get_realtime());

    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);
    const unsigned long old_count_timeout = grid_state.count_timeout;
    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : resetFPUs() error: driver is not connected, can't reset FPUs\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }

    // make sure no FPU is moving or finding datum
    bool resetok=true;
    for (int i=0; i < config.num_fpus; i++)
    {
        if (! fpuset[i])
        {
            continue;
        }
        t_fpu_state& fpu_state = grid_state.FPU_state[i];
        // we exclude moving FPUs snf FPUs which are
        // searching datum.
        if ( (fpu_state.state == FPST_MOVING)
                && (fpu_state.state == FPST_DATUM_SEARCH))
        {
            resetok = false;
        }
    }

    if (! resetok)
    {
        // We do not perform a reset when there are moving FPUs.  (In
        // that case, the user should send an abortMotion command
        // first.)
        LOG_CONTROL(LOG_ERROR, "%18.6f : error: FPUs are moving, refusing to reset FPUs. Call abortMotion first.\n",
                    canlayer::get_realtime());
        return DE_STILL_BUSY;
    }


    int cnt_pending=0;
    unique_ptr<ResetFPUCommand> can_command;
    for (int i=0; i < config.num_fpus; i++)
    {
        if (!fpuset[i])
        {
            continue;
        }
        bool broadcast = false;
        can_command = gateway.provideInstance<ResetFPUCommand>();
        can_command->parametrize(i, broadcast);
        unique_ptr<I_CAN_Command> cmd(can_command.release());
        gateway.sendCommand(i, cmd);
        cnt_pending++;
    }


    while ( (cnt_pending > 0) && ((grid_state.driver_state == DS_CONNECTED)))
    {
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        cnt_pending = (grid_state.count_pending + grid_state.num_queued);
    }


    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : error: driver is not connected, can't reset FPUs\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }

    // It is important to compare for inequality here, because
    // count_timeout is an unsigned value which can intentionally wrap
    // without causing undefined behaviour.
    if (grid_state.count_timeout != old_count_timeout)
    {
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    LOG_CONTROL(LOG_INFO, "%18.6f : resetFPUs: command completed succesfully\n",
                canlayer::get_realtime());

    logGridState(config.logLevel, grid_state);

    return DE_OK;

}

void AsyncDriver::getFPUsetOpt(t_fpuset const * const fpuset_opt, t_fpuset &fpuset) const
{
    if (fpuset_opt != nullptr)
    {
        memcpy(fpuset, fpuset_opt, sizeof(fpuset));
    }
    else
    {
        for(int i= 0; i < config.num_fpus; i++)
        {
            fpuset[i] = true;
        }
    }
}

E_DriverErrCode AsyncDriver::startAutoFindDatumAsync(t_grid_state& grid_state,
        E_GridState& state_summary,
        E_DATUM_SEARCH_DIRECTION * p_direction_flags,
        E_DATUM_SELECTION arm_selection,
        E_DATUM_TIMEOUT_FLAG timeout_flag,
        bool count_protection,
        t_fpuset const * const fpuset_opt)
{

    t_fpuset fpuset;
    getFPUsetOpt(fpuset_opt, fpuset);

    {

        const char * to_string;
        switch (timeout_flag)
        {
        case DATUM_TIMEOUT_ENABLE:
            to_string = "enabled";
            break;
        case DATUM_TIMEOUT_DISABLE:
            to_string = "disabled";
            break;
        default:
            LOG_CONTROL(LOG_ERROR, "%18.6f : findDatum(): error: invalid time-out setting\n",
                        canlayer::get_realtime());
            return DE_INVALID_PAR_VALUE;
        }




        const char * as_string;
        switch (arm_selection)
        {
        case DASEL_BOTH:
            as_string = "'both arms'";
            break;
        case DASEL_ALPHA:
            as_string = "'alpha arm'";
            break;

        case DASEL_BETA:
            as_string = "'beta arm'";
            break;

        case DASEL_NONE:
            as_string = "(no arm selected)";
            break;

        default:
            as_string = "'invalid selection'";
            LOG_CONTROL(LOG_ERROR, "%18.6f : findDatum(): error: invalid arm selection\n",
                        canlayer::get_realtime());
            return DE_INVALID_PAR_VALUE;
        }

        LOG_CONTROL(LOG_INFO, "%18.6f : AsyncDriver: findDatum started, arm_selection=%s, timeouts=%s\n",
                    canlayer::get_realtime(), as_string, to_string);
    }

    bool contains_auto=false;
    bool contains_anti_clockwise=false;
    bool timeouts_disabled = (timeout_flag == DATUM_TIMEOUT_DISABLE);

    // if present, copy direction hint
    t_datum_search_flags direction_flags;
    if (p_direction_flags == nullptr)
    {
        for(int i=0; i < config.num_fpus; i++)
        {
            if (fpuset[i])
            {
                direction_flags[i] = SEARCH_AUTO;
            }
            else
            {
                direction_flags[i] = SKIP_FPU;
            }
        }
    }
    else
    {
        for(int i=0; i < config.num_fpus; i++)
        {
            if (fpuset[i])
            {
                direction_flags[i] = p_direction_flags[i];
            }
            else
            {
                direction_flags[i] = SKIP_FPU;
            }
        }
    }

    for(int i=0; i < config.num_fpus; i++)
        // check search direction
    {
        if (! fpuset[i])
        {
            continue;
        }
        const char * as_string;
        switch (direction_flags[i])
        {
        case SEARCH_CLOCKWISE:
            as_string = "'clockwise'";
            break;
        case SEARCH_ANTI_CLOCKWISE:
            contains_anti_clockwise=true;
            as_string = "'anti-clockwise'";
            break;

        case SEARCH_AUTO:
            contains_auto=true;
            as_string = "'automatic'";
            break;

        case SKIP_FPU:
            as_string = "'skip FPU'";
            break;

        default:
            LOG_CONTROL(LOG_ERROR, "%18.6f : findDatum(): error: invalid direction selection '%i' for FPU #%i\n",
                        canlayer::get_realtime(), direction_flags[i], i);
            return DE_INVALID_PAR_VALUE;
        }

        if (contains_auto && timeouts_disabled)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : findDatum(): error: time-outs disabled, but automatic search selected for FPU #%i\n",
                        canlayer::get_realtime(), i);
            return DE_INVALID_PAR_VALUE;
        }
        LOG_CONTROL(LOG_INFO, "%18.6f : AsyncDriver: findDatum(): direction selection for FPU %i =%s\n",
                    canlayer::get_realtime(), i, as_string);
    }


    // We need a version check if the beta arm is moved
    // anti-clockwise or automatically.
    bool fw_version_check_needed = (((arm_selection == DASEL_BOTH)
                                     || (arm_selection == DASEL_BETA))
                                    && (contains_auto
                                        || contains_anti_clockwise));

    // We also need a version check if the beta arm
    // is not moved, because earlier firmware versions would
    // move it anyway, ignoring the instruction.

    fw_version_check_needed |= ((arm_selection == DASEL_ALPHA)
                                || (arm_selection == DASEL_NONE));

    // capability to disable time-outs is for firmware versions >= 1.4.3
    fw_version_check_needed |= timeouts_disabled;

    uint8_t min_firmware_version[3] = {0,0,0};
    int min_firmware_fpu = -1;

    E_DriverErrCode ecode = getMinFirmwareVersion(fpuset, min_firmware_version, min_firmware_fpu, grid_state, state_summary);


    if (fw_version_check_needed)
    {
        const int req_fw_major = 1;
        int req_fw_minor = 0;
        int req_fw_patch = 0;
        // The following checks need to be in ascending order,
        // so that the maximum is found.
        if ((arm_selection == DASEL_ALPHA) || (arm_selection == DASEL_NONE))
        {
            req_fw_minor = 1;
        }
        if (contains_anti_clockwise)
        {
            req_fw_minor = 2;
        }
        else if (contains_auto)
        {
            req_fw_minor = 2;
        }
        if (timeouts_disabled)
        {
            req_fw_minor = 4;
            req_fw_patch = 3;
        }
        if ( (min_firmware_version[0] < req_fw_major)
                || (min_firmware_version[1] < req_fw_minor)
                || (min_firmware_version[2] < req_fw_patch))
        {
            // the firmware does not implement what we need
            LOG_CONTROL(LOG_ERROR, "%18.6f : findDatum(): error: DE_FIRMWARE_UNIMPLEMENTED"
                        " command requires firmware version >= %i.%i.%i,"
                        " version %i.%i.%i found in FPU %i\n",
                        canlayer::get_realtime(),
                        req_fw_major, req_fw_minor, req_fw_patch,
                        min_firmware_version[0], min_firmware_version[1], min_firmware_version[2], min_firmware_fpu);
            return DE_FIRMWARE_UNIMPLEMENTED;
        }

    }

    // now, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);
    const unsigned long old_count_timeout = grid_state.count_timeout;
    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : findDatum(): error: driver is not connected\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }

    // check for valid arm selection
    switch (arm_selection)
    {
    case DASEL_BOTH:
    case DASEL_ALPHA:
    case DASEL_BETA:
    case DASEL_NONE:
        break;
    default:
        LOG_CONTROL(LOG_ERROR, "%18.6f : findDatum(): error: invalid arm selection '%i'\n",
                    canlayer::get_realtime(), int(arm_selection));
        return DE_INVALID_PAR_VALUE;
    }

    ecode = pingFPUsAsync(grid_state, state_summary, fpuset);

    if (ecode != DE_OK)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : ping failed, aborting findDatum()operation \n",
                    canlayer::get_realtime());
        return ecode;
    }

    // check no FPUs have ongoing collisions
    for (int i=0; i < config.num_fpus; i++)
    {
        if (! fpuset[i])
        {
            continue;
        }
        const t_fpu_state fpu = grid_state.FPU_state[i];
        const E_FPU_STATE fpu_status = fpu.state;
        if (fpu_status == FPST_OBSTACLE_ERROR)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : unresolved collision for FPU ## %i - aborting findDatum()operation.\n",
                        canlayer::get_realtime(), i);
            return DE_UNRESOLVED_COLLISION;
        }
        if (fpu_status == FPST_ABORTED)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : FPU #%i is in aborted state - cancelling findDatum()operation.\n",
                        canlayer::get_realtime(), i);
            return DE_IN_ABORTED_STATE;
        }

        if ( (fpu.alpha_datum_switch_active) && ((arm_selection == DASEL_ALPHA) || (arm_selection == DASEL_BOTH)))
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : FPU #%i has active alpha datum/limit switch"
                        " - cancelling findDatum()operation.\n",
                        canlayer::get_realtime(), i);
            // We differentiate the error codes in one for the above pre-check, and
            // another for the firmware error message. This makes it possible
            // to derive the correct arm location in the protection layer.
            return DE_ALPHA_ARM_ON_LIMIT_SWITCH;
        }
    }

    // check that beta arms are in allowed half-plane
    if ((arm_selection == DASEL_BETA) || (arm_selection == DASEL_BOTH))
    {
        for (int i=0; i < config.num_fpus; i++)
        {
            if (!fpuset[i])
            {
                continue;
            }
            const int BETA_DATUM_LIMIT = -5 * STEPS_PER_DEGREE_BETA;
            int beta_steps = grid_state.FPU_state[i].beta_steps;
            bool beta_initialized = grid_state.FPU_state[i].beta_was_zeroed;

            E_DATUM_SEARCH_DIRECTION beta_mode = direction_flags[i];

            if (count_protection && (beta_steps < BETA_DATUM_LIMIT) && (beta_mode == SEARCH_CLOCKWISE))
            {
                LOG_CONTROL(LOG_ERROR, "%18.6f : findDatum():"
                            " FPU %i: beta arm appears to be in unsafe negative position < -5 degree"
                            "and mode is SEARCH_CLOCKWISE - aborting findDatum() operation \n",
                            canlayer::get_realtime(), i);
                return DE_PROTECTION_ERROR;
            }

            if (count_protection && (beta_steps > 0) && (beta_mode == SEARCH_ANTI_CLOCKWISE))
            {
                LOG_CONTROL(LOG_ERROR, "%18.6f : findDatum():"
                            " FPU %i: beta arm appears to be in positive position "
                            "and mode is SEARCH_ANTI_CLOCKWISE - aborting findDatum() operation \n",
                            canlayer::get_realtime(), i);
                return DE_PROTECTION_ERROR;
            }

            if (count_protection && (! beta_initialized) && (beta_mode == SEARCH_AUTO))
            {
                LOG_CONTROL(LOG_ERROR, "%18.6f : findDatum():"
                            " FPU %i beta arm is uninitialized "
                            "and mode is SEARCH_AUTO - aborting findDatum() operation \n",
                            canlayer::get_realtime(), i);
                return DE_PROTECTION_ERROR;
            }

        }
    }

    // All fpus which are allowed to move, are moved automatically
    // until they hit the datum switch.

    unique_ptr<FindDatumCommand> can_command;
    for (int i=0; i < config.num_fpus; i++)
    {
        // FPUs which are not in the set are skipped
        if ((direction_flags[i] == SKIP_FPU) || (! fpuset[i]))
        {
            continue;
        }

        t_fpu_state& fpu_state = grid_state.FPU_state[i];
        if ( (fpu_state.state != FPST_UNINITIALIZED)
                || (fpu_state.state != FPST_AT_DATUM)
                || (fpu_state.state != FPST_READY_FORWARD)
                || (fpu_state.state != FPST_READY_REVERSE)
                || (fpu_state.state != FPST_RESTING))

        {

            bool broadcast = false;
            can_command = gateway.provideInstance<FindDatumCommand>();

            can_command->parametrize(i, broadcast, direction_flags[i], arm_selection, timeout_flag);
            unique_ptr<I_CAN_Command> cmd(can_command.release());
            gateway.sendCommand(i, cmd);

        }
    }


    // It is important to compare for inequality here, because
    // count_timeout is an unsigned value which can intentionally wrap
    // without causing undefined behaviour.
    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : findDatum(): error: command timed out\n",
                    canlayer::get_realtime());
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    LOG_CONTROL(LOG_INFO, "%18.6f : findDatum(): command successfully sent\n",
                canlayer::get_realtime());

#if CAN_PROTOCOL_VERSION == 1
    // we store the parameter here because it is not echoed in protocol 1
    last_datum_arm_selection = arm_selection;
#endif

    log_repeat_count =0; // adjust frequency of logging
    return DE_OK;

}


bool p_repeat_log(unsigned int &log_repeat_count)
{
    unsigned int lrc = log_repeat_count++;

    if (lrc <= 10)
    {
        return true;
    }
    else if (lrc <= 50)
    {
        return ((lrc % 5) == 0);
    }
    else if (lrc <= 100)
    {
        return ((lrc % 10) == 0);
    }
    else if (lrc <= 500)
    {
        return ((lrc % 50) == 0);
    }
    else
    {
        return (lrc % 100) == 0;
    }

}

E_DriverErrCode AsyncDriver::waitAutoFindDatumAsync(t_grid_state& grid_state,
        E_GridState& state_summary,
        double &max_wait_time, bool &finished, t_fpuset const * const fpuset_opt)
{
    t_fpuset fpuset;
    getFPUsetOpt(fpuset_opt, fpuset);

    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : waitFindDatum():  error DE_NO_CONNECTION, connection was lost\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }

#if (CAN_PROTOCOL_VERSION == 1)
    const t_grid_state prev_grid_state = grid_state;
#endif

    bool cancelled = false;


    const unsigned long old_count_timeout = grid_state.count_timeout;

    state_summary = gateway.waitForState(TGT_NO_MORE_MOVING,
                                         grid_state, max_wait_time, cancelled);

    int num_moving = (grid_state.Counts[FPST_DATUM_SEARCH]
                      + grid_state.count_pending
                      + grid_state.num_queued);

    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : waitFindDatum(): error: driver is not connected\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }

    for (int i=0; i < config.num_fpus; i++)
    {

        t_fpu_state fpu = grid_state.FPU_state[i];
        E_FPU_STATE fpu_status = fpu.state;

        if (fpu_status == FPST_OBSTACLE_ERROR)
        {
            if (fpu.beta_collision)
            {
                LOG_CONTROL(LOG_ERROR, "%18.6f : waitFindDatum(): error: collision detected for FPU %i\n",
                            canlayer::get_realtime(), i);
                logGridState(config.logLevel, grid_state);
                fsync(config.fd_controllog);

                return DE_NEW_COLLISION;
            }
            else
            {
                LOG_CONTROL(LOG_ERROR, "%18.6f : waitFindDatum(): error: limit breach detected for FOU %i\n",
                            canlayer::get_realtime(), i);
                logGridState(config.logLevel, grid_state);
                fsync(config.fd_controllog);

                return DE_NEW_LIMIT_BREACH;
            }
        }
        if (fpu_status == FPST_ABORTED)
        {

            if (fpu.last_status == ER_DATUMTO)
            {
                LOG_CONTROL(LOG_ERROR, "%18.6f : waitFindDatum(): CRITICAL ERROR: Datum operation timed out for FPU %i\n",
                            canlayer::get_realtime(), i);
                logGridState(config.logLevel, grid_state);
                fsync(config.fd_controllog);

                return DE_DATUM_COMMAND_HW_TIMEOUT;
            }
            else
            {
                LOG_CONTROL(LOG_ERROR, "%18.6f : waitFindDatum(): error: FPU movement was aborted for FPU %i\n",
                            canlayer::get_realtime(), i);
                logGridState(config.logLevel, grid_state);
                fsync(config.fd_controllog);

                return DE_MOVEMENT_ABORTED;
            }
        }

    }

    // we do this check in a new loop with the goal to give collision reports precedence
    for (int i=0; i < config.num_fpus; i++)
    {

        t_fpu_state fpu = grid_state.FPU_state[i];
        E_FPU_STATE fpu_status = fpu.state;

        if ((fpu_status == FPST_UNINITIALIZED) && (fpu.last_status == ER_DATUM_LIMIT))
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : waitFindDatum(): error: FPU %i alpha arm on datum switch, movement rejected\n",
                        canlayer::get_realtime(), i);
            logGridState(config.logLevel, grid_state);
            fsync(config.fd_controllog);

            return DE_HW_ALPHA_ARM_ON_LIMIT_SWITCH;
        }

    }


    if (state_summary == GS_COLLISION)
    {
        printf("collision detected, aborting datum search");
        logGridState(config.logLevel, grid_state);
        fsync(config.fd_controllog);

        return DE_NEW_COLLISION;
    }

    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : waitFindDatum(): error: command timed out\n",
                    canlayer::get_realtime());

        logGridState(config.logLevel, grid_state);
        fsync(config.fd_controllog);

        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    finished = (num_moving == 0) && (! cancelled);


    for(int i=0; i < config.num_fpus; i++)
    {
        if (! fpuset[i])
        {
            continue;
        }
        if ((grid_state.FPU_state[i].state == FPST_UNINITIALIZED)
                && (grid_state.FPU_state[i].last_status == ER_AUTO))
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : findDatum(): error: DE_PROTECTION_ERROR, FPU denied automatic datum search\n",
                        canlayer::get_realtime());
            return DE_PROTECTION_ERROR;
        }
    }


#if (CAN_PROTOCOL_VERSION == 1)
    {
        // Check if command moved both arms without instruction.
        //
        // In this case, we have firmware 1.0 on some FPUs, which will
        // trigger a warning because unexpected moves can break the
        // beta arm.
        bool move_alpha = (last_datum_arm_selection == DASEL_BOTH) || (last_datum_arm_selection == DASEL_ALPHA);
        bool move_beta = (last_datum_arm_selection == DASEL_BOTH) || (last_datum_arm_selection == DASEL_BETA);

        for(int i = 0; i < config.num_fpus; i++)
        {
            if ((((prev_grid_state.FPU_state[i].alpha_steps != 0)
                    && (grid_state.FPU_state[i].alpha_steps == 0)
                    && (! move_alpha))
                )
                    ||
                    ( ((prev_grid_state.FPU_state[i].beta_steps != 0)
                       && (grid_state.FPU_state[i].beta_steps == 0)
                       && (! move_beta))
                    ))
            {
                fprintf(stderr, "AsyncDriver: WARNING\b: findDatum() moved both arms in FPU %i,"
                        " arm selection ignored, this indicates version 1.0 firmware on FPU.\n",
                        i);
                LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncDriver: WARNING: findDatum moved both"
                            " arms in FPU %i, arm selection ignored.\n",
                            canlayer::get_realtime(), i);
            }
        }
    }
#endif

    if (finished)
    {
        LOG_CONTROL(LOG_INFO, "%18.6f : AsyncDriver: findDatum finished successfully\n",
                    canlayer::get_realtime());

        logGridState(config.logLevel, grid_state);
        fsync(config.fd_controllog);

        return DE_OK;
    }
    else
    {
        if (p_repeat_log(log_repeat_count))
        {
            LOG_CONTROL(LOG_GRIDSTATE, "%18.6f : AsyncDriver: findDatum not finished, waiting time elapsed\n",
                        canlayer::get_realtime());
            if (config.logLevel >= LOG_VERBOSE)
            {
                logGridState(config.logLevel, grid_state);
            }
        }
        fsync(config.fd_controllog);

        return DE_WAIT_TIMEOUT;
    }


}

E_DriverErrCode AsyncDriver::validateWaveforms(const t_wtable& waveforms,
        const int MIN_STEPS,
        const int MAX_STEPS,
        const int MAX_START_STEPS,
        const unsigned int MAX_NUM_SECTIONS,
        const double MAX_INCREASE_FACTOR) const
{

    LOG_CONTROL(LOG_INFO, "%18.6f : AsyncDriver: validating waveforms\n",
                canlayer::get_realtime());

    const int num_loading =  waveforms.size();
    const unsigned int num_steps = waveforms[0].steps.size();

    if (MIN_STEPS > MAX_STEPS)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncDriver: error DE_INVALID_CONFIG:"
                    "  minimum step number limit is larger than maximum limit\n",
                    canlayer::get_realtime());
        return DE_INVALID_CONFIG;
    }
    if (MAX_START_STEPS > MAX_STEPS)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncDriver: error DE_INVALID_CONFIG:"
                    " upper limit of step count during start exceeds maximum step count\n",
                    canlayer::get_realtime());
        return DE_INVALID_CONFIG;
    }
    if (MAX_START_STEPS <= MIN_STEPS)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncDriver: error DE_INVALID_CONFIG:"
                    " upper limit of step count during start is smaller than minimum value\n",
                    canlayer::get_realtime());
        return DE_INVALID_CONFIG;
    }
    if (MAX_INCREASE_FACTOR < 1.0)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncDriver: error DE_INVALID_CONFIG:"
                    " relative growth factor is smaller than 1.\n",
                    canlayer::get_realtime());
        return DE_INVALID_CONFIG;
    }


    if (num_steps > MAX_NUM_SECTIONS)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncDriver: error DE_INVALID_WAVEFORM_TOO_MANY_SECTIONS:"
                    "  waveform has too many steps (%i)\n",
                    canlayer::get_realtime(), num_steps);
        return DE_INVALID_WAVEFORM_TOO_MANY_SECTIONS;
    }

    for (int fpu_index=0; fpu_index < num_loading; fpu_index++)
    {
        const int fpu_id = waveforms[fpu_index].fpu_id;
        if ((fpu_id >= config.num_fpus) || (fpu_id < 0))
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncDriver: waveform error DE_INVALID_FPU_ID:"
                        " FPU ID %i in waveform table is out of range\n",
                        canlayer::get_realtime(), fpu_id);
            return DE_INVALID_FPU_ID;
        }

        // require same number of steps for all FPUs
        if (waveforms[fpu_index].steps.size() != num_steps)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncDriver: error DE_INVALID_WAVEFORM_RAGGED:"
                        " waveforms for FPU %i have unequal length\n",
                        canlayer::get_realtime(), fpu_id);
            return DE_INVALID_WAVEFORM_RAGGED;
        }


        const t_waveform& wform = waveforms[fpu_index];

        for(int chan_idx=0; chan_idx < 2; chan_idx++)
        {
            int xa_last = 0;
            int x_last_sign = 0;

            for (unsigned int sidx=0; sidx<num_steps; sidx++)
            {

                const t_step_pair& step = wform.steps[sidx];

                const int xs = (chan_idx == 0) ?
                               step.alpha_steps : step.beta_steps;

                const int x_sign = (xs > 0) ? 1 : ((xs < 0) ? -1: 0);
                const int xa = abs(xs);

                // absolute value of step count of next entry, or zero if at end
                const int xa_next = ( (sidx == (num_steps -1))
                                      ? 0
                                      :  abs(((chan_idx == 0)
                                              ? wform.steps[sidx+1].alpha_steps
                                              : wform.steps[sidx+1].beta_steps)));

                //printf("fpu %i: channel=%i, step=%i, xs=%i, xa=%i", fpu_id, chan_idx, sidx, xs, xa);

                if (xa > MAX_STEPS)
                {
                    LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncDriver: error DE_INVALID_WAVEFORM_STEPCOUNT_TOO_LARGE:"
                                "fpu %i, %s arm, movement interval %i: step count exceeds maximum\n\n",
                                canlayer::get_realtime(),
                                fpu_id, chan_idx == 0 ? "alpha" : "beta", sidx);
                    return DE_INVALID_WAVEFORM_STEPCOUNT_TOO_LARGE;
                }



                const int xa_small = std::min(xa_last, xa);
                const int xa_large = std::max(xa_last, xa);
                const int increase_limit = int(ceil(xa_small * MAX_INCREASE_FACTOR));

                const bool valid_acc = (
                                           // 1) movement into the same direction

                                           ((x_sign == x_last_sign)
                                            //   1a) and currently *stopping* to move
                                            && (( (xa < MIN_STEPS)
                                                  && (xa_last <= MAX_START_STEPS))
                                                // or, 1b) at least  MIN_STEPS and the larger
                                                // of both values not larger than the allowed
                                                // relative increase
                                                || ( (xa_small >= MIN_STEPS)
                                                     && (xa_large <= increase_limit))))
                                           // or, has stopped to move (and only in this case,
                                           // the step count can be smaller than MIN_STEPS)
                                           || ( (xa == 0)
                                                && (xa_last < MAX_START_STEPS))
                                           // or, a single entry with a small number of steps,
                                           // followed by a pause or end of the table
                                           || ( (xa <= MAX_START_STEPS)
                                                && (xa_last == 0)
                                                && (xa_next == 0))
                                           // or, with or without a change of direction,
                                           // one step number zero and the other below or at
                                           // MAX_START_STEPS - at start or end of a movement
                                           || ((xa_small == 0)
                                               && (xa_large <= MAX_START_STEPS))
                                           // or, a pause in movement
                                           || ((xa_small == 0)
                                               && (xa_large == 0)));

                if (!valid_acc)
                {
                    LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncDriver: DE_INVALID_WAVEFORM_CHANGE: "
                                "fpu %i, %s arm, movement interval %i: invalid step count change\n",
                                canlayer::get_realtime(),
                                fpu_id, chan_idx == 0 ? "alpha" : "beta", sidx);
                    return DE_INVALID_WAVEFORM_CHANGE;
                }

                xa_last = xa;
                x_last_sign = x_sign;
            }
            if (xa_last > MAX_START_STEPS)
            {
                // last step count must be minimum or smaller
                LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncDriver: DE_INVALID_WAVEFORM_TAIL: "
                            "fpu %i, %s arm, movement interval %i: last step count too large\n",
                            canlayer::get_realtime(),
                            fpu_id, chan_idx == 0 ? "alpha" : "beta", num_steps -1);
                return DE_INVALID_WAVEFORM_TAIL;
            }
        }
    }
    LOG_CONTROL(LOG_INFO, "%18.6f : AsyncDriver::validateWaveforms() waveform OK\n",
                canlayer::get_realtime());

    return DE_OK;
}

E_DriverErrCode AsyncDriver::configMotionAsync(t_grid_state& grid_state,
        E_GridState& state_summary,
        const t_wtable& waveforms,
        t_fpuset const &fpuset,
        bool soft_protection,
        bool allow_uninitialized)
{

    LOG_CONTROL(LOG_INFO, "%18.6f : AsyncDriver: calling configMotion()\n",
                canlayer::get_realtime());

    // first, get current state of the grid
    state_summary = gateway.getGridState(grid_state);

#if (CAN_PROTOCOL_VERSION != 1 )
    // FIXME: disable checks for time-outs for now
    const unsigned long old_count_timeout = grid_state.count_timeout;
#endif


    const int min_stepcount = int(floor(config.motor_minimum_frequency
                                        * WAVEFORM_SEGMENT_DURATION_MS  / 1000));

    // perform hardware protection checks unless
    // explicitly disabled.

    // check no FPUs have ongoing collisions
    // and all have been initialized
    for (int i=0; i < config.num_fpus; i++)
    {
        if (soft_protection)
        {
            E_FPU_STATE fpu_status = grid_state.FPU_state[i].state;
            if (fpu_status == FPST_OBSTACLE_ERROR)
            {
                LOG_CONTROL(LOG_ERROR, "%18.6f : configMotion(): error DE_UNRESOLVED_COLLISION"
                            " - unresolved collision active for FPU %i\n",
                            canlayer::get_realtime(), i);
                return DE_UNRESOLVED_COLLISION;
            }
            // This isn't enforced in protocol version 1,
            // because we do not have an enableMove command.
            // In protocol version 2, the user has to issue
            // enableMove first.
            if (fpu_status == FPST_ABORTED)
            {
                LOG_CONTROL(LOG_ERROR, "%18.6f : configMotion(): error DE_ABORTED_STATE"
                            " - FPU %i is in aborted state\n",
                            canlayer::get_realtime(), i);
                return DE_IN_ABORTED_STATE;
            }
        }

        if (!allow_uninitialized)
        {
            if ( ! (grid_state.FPU_state[i].alpha_was_zeroed
                    && grid_state.FPU_state[i].beta_was_zeroed))
            {
                LOG_CONTROL(LOG_ERROR, "%18.6f : configMotion(): error DE_FPUS_NOT_CALIBRATED"
                            " - FPU %i is not calibrated and soft_protection flag was not cleared\n",
                            canlayer::get_realtime(), i);
                return DE_FPUS_NOT_CALIBRATED;
            }
        


	    const int max_stepcount = int(ceil(config.motor_maximum_frequency
					       * WAVEFORM_SEGMENT_DURATION_MS  / 1000));
	    const int max_start_stepcount = int(ceil(config.motor_max_start_frequency
						     * WAVEFORM_SEGMENT_DURATION_MS  / 1000));

	    const double max_rel_increase = config.motor_max_rel_increase;

	    const E_DriverErrCode vwecode = validateWaveforms(waveforms,
							      min_stepcount,
							      max_stepcount,
							      max_start_stepcount,
							      ConfigureMotionCommand::MAX_NUM_SECTIONS,
							      max_rel_increase);
	    if (vwecode != DE_OK)
	    {
		return vwecode;
	    }
	}
    }


    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : configMotion(): error DE_NO_CONNECTION - no connection present\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }

    unique_ptr<ConfigureMotionCommand> can_command;
    // loop over number of steps in the table
    const int num_steps = waveforms[0].steps.size();
#if (CAN_PROTOCOL_VERSION == 1)
    bool configured_fpus[MAX_NUM_POSITIONERS];
    memset(configured_fpus, 0, sizeof(configured_fpus));
#endif
    const int num_loading =  waveforms.size();

    int step_index = 0;
    int retry_downcount = 5;
    int alpha_cur[MAX_NUM_POSITIONERS];
    int beta_cur[MAX_NUM_POSITIONERS];
    while (step_index < num_steps)
    {
        const bool first_entry = (step_index == 0);
        const bool last_entry = (step_index == (num_steps-1));
        if (first_entry)
        {
            // get current step number to track positions
            memset(alpha_cur,0,sizeof(alpha_cur));
            memset(beta_cur,0,sizeof(beta_cur));
            for (int i = 0; i < config.num_fpus; i++)
            {
                alpha_cur[i] = grid_state.FPU_state[i].alpha_steps;
                beta_cur[i] = grid_state.FPU_state[i].beta_steps;
            }
        }


        for (int fpu_index=0; fpu_index < num_loading; fpu_index++)
        {

            if ((fpu_index == 0) && (step_index != 0))
            {
                // Wait a short time before talking to the same FPU again because the FPUs seem to be
                // in general a bit sluggish.
                // We don't care about signals here.
                usleep(ConfigureMotionCommand::CHAT_PAUSE_TIME_USEC);
            }
            int fpu_id = waveforms[fpu_index].fpu_id;
            if ((fpu_id >= config.num_fpus) || (fpu_id < 0))
            {
                // the FPU id is out of range
                LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncDriver::configMotion(): FPU id '%i' is out of range"
                            "needs to be between 0 and %i\n",
                            canlayer::get_realtime(),
                            fpu_id, (config.num_fpus - 1));
                return DE_INVALID_FPU_ID;
            }

            if (! fpuset[fpu_id])
            {
                continue;
            }

            t_fpu_state& fpu_state = grid_state.FPU_state[fpu_id];
            if (fpu_state.state != FPST_LOCKED)
            {
                // get a command buffer
                can_command = gateway.provideInstance<ConfigureMotionCommand>();

                const t_step_pair& step = waveforms[fpu_index].steps[step_index];

                can_command->parametrize(fpu_id,
                                         step.alpha_steps,
                                         step.beta_steps,
                                         first_entry,
                                         last_entry,
                                         min_stepcount);

                // send the command (the actual sending happens
                // in the TX thread in the background).
                unique_ptr<I_CAN_Command> cmd(can_command.release());
                alpha_cur[fpu_id] += step.alpha_steps;
                beta_cur[fpu_id] += step.beta_steps;

                LOG_CONTROL(LOG_VERBOSE, "%18.6f : configMotion(): sending wtable section %i, fpu # %i "
                            "= (%+4i, %+4i) steps --> pos (%7.3f, %7.3f) degree)\n",
                            canlayer::get_realtime(),
                            step_index, fpu_id, step.alpha_steps, step.beta_steps,
                            (alpha_cur[fpu_id] / STEPS_PER_DEGREE_ALPHA) + config.alpha_datum_offset,
                            beta_cur[fpu_id] / STEPS_PER_DEGREE_BETA);

                gateway.sendCommand(fpu_id, cmd);
            }
        }
#if (CAN_PROTOCOL_VERSION != 1)
        /* Apparently, at least for firmware version 1, we cannot
               send more than one configMotion command at a time,
               or else CAN commands will get lost. */
        if (first_entry || last_entry)
#endif
        {
            /* Wait and check that all FPUs are registered in LOADING
               state.  This is needed to make sure we have later a clear
               state transition for finishing the load with the last
               flag set. */
            double max_wait_time = -1;
            bool cancelled = false;
            state_summary = gateway.waitForState(TGT_NO_MORE_PENDING,
                                                 grid_state, max_wait_time, cancelled);
            if (grid_state.driver_state != DS_CONNECTED)
            {
                LOG_CONTROL(LOG_ERROR, "%18.6f : configMotion(): error: driver is not connected\n",
                            canlayer::get_realtime());


                return DE_NO_CONNECTION;
            }
            bool do_retry = false;
            //int num_loading =  waveforms.size();
            for (int fpu_index=0; fpu_index < num_loading; fpu_index++)
            {
                int fpu_id = waveforms[fpu_index].fpu_id;

                if (! fpuset[fpu_id])
                {
                    continue;
                }

                t_fpu_state& fpu_state = grid_state.FPU_state[fpu_id];
                // we retry if an FPU which we tried to configure and is
                // not locked did not change to FPST_LOADING state.
                if ((fpu_state.state != FPST_LOCKED)
                        && ( ((first_entry && (! last_entry))
                              &&  (fpu_state.state != FPST_LOADING))
                             || (last_entry
                                 &&  (fpu_state.state != FPST_READY_FORWARD))))
                {
                    if (retry_downcount <= 0)
                    {
                        return DE_MAX_RETRIES_EXCEEDED;
                    }
                    do_retry = true;
                    LOG_CONTROL(LOG_INFO, "%18.6f : configMotion(): warning: "
                                "loading/ready state not confirmed for FPU #%i,"
                                " retry from start! (%i retries left)\n",
                                canlayer::get_realtime(),
                                fpu_id,
                                retry_downcount);
                }
            }
            if (do_retry)
            {
                // we start again with loading the first step
                step_index = 0;
                retry_downcount--;
                continue;
            }

        }
        step_index++;
    }

#if (CAN_PROTOCOL_VERSION != 1 )
    // seems not to work reliably with current firmware
    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : configMotion(): error: CAN command timed out\n",
                    canlayer::get_realtime());

        logGridState(config.logLevel, grid_state);

        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }
#endif


    for (int fpu_index=0; fpu_index < num_loading; fpu_index++)
    {
        int fpu_id = waveforms[fpu_index].fpu_id;

        if (! fpuset[fpu_id])
        {
            continue;
        }

        LOG_CONTROL(LOG_GRIDSTATE, "%18.6f : configMotion(): fpu # %i "
                    "--> pos (%5i, %5i) steps ~ (%+9.3f, %+9.3f) degree) - OK\n",
                    canlayer::get_realtime(),
                    fpu_id,
                    alpha_cur[fpu_id],
                    beta_cur[fpu_id],
                    (alpha_cur[fpu_id] / STEPS_PER_DEGREE_ALPHA) + config.alpha_datum_offset,
                    beta_cur[fpu_id] / STEPS_PER_DEGREE_BETA);
    }

    LOG_CONTROL(LOG_INFO, "%18.6f : configMotion(): waveforms successfully sent OK\n",
                canlayer::get_realtime());

    logGridState(config.logLevel, grid_state);

    return DE_OK;
}

#if 0
E_DriverErrCode AsyncDriver::configPathsAsync(t_grid_state& grid_state,
					      E_GridState& state_summary,
					      const t_wtable& waveforms,
					      t_fpuset const &fpuset,
					      bool soft_protection,
					      bool allow_uninitialized)
{
    // convert path using the selected gearbox correction
    // round to whole steps
    // check that first position is current position -> if not, error code
    // compute differences
    // store rounded paths to waveform
    // call configMotion, and return result
    
}
#endif

E_DriverErrCode AsyncDriver::startExecuteMotionAsync(t_grid_state& grid_state,
        E_GridState& state_summary,
        t_fpuset const &fpuset)
{

    LOG_CONTROL(LOG_VERBOSE, "%18.6f : AsyncDriver: starting executeMotion()\n",
                canlayer::get_realtime());

    // first, get current state of the grid
    state_summary = gateway.getGridState(grid_state);
    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : executeMotion(): error DE_NO_CONNECTION, driver is not connected\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }

    // check no FPUs have ongoing collisions
    for (int i=0; i < config.num_fpus; i++)
    {
        if (!fpuset[i])
        {
            continue;
        }
        E_FPU_STATE fpu_status = grid_state.FPU_state[i].state;

        if (fpu_status == FPST_OBSTACLE_ERROR)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : executeMotion(): error DE_UNRESOLVED_COLLISION in PU %i, ongoing collision\n",
                        canlayer::get_realtime(), i);
            return DE_UNRESOLVED_COLLISION;
        }
        if (fpu_status == FPST_ABORTED)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : executeMotion(): error DE_ABORTED_STATE in FPU %i, FPUs are in aborted state\n",
                        canlayer::get_realtime(), i);
            return DE_IN_ABORTED_STATE;
        }

    }


    int num_moving = 0; // Number of FPUs which will move
    bool use_broadcast = true; // flag whether we can use a fast broadcast command

    /* check all FPUs in READY_* state have valid waveforms
       This check intends to make sure that even in protocol version 1,
       waveforms are not used when they have been involved
       in collision or abort. */
    for (int i=0; i < config.num_fpus; i++)
    {
        if (!fpuset[i])
        {
            // we need to send the command individually
            use_broadcast = false;
            continue;
        }

        E_FPU_STATE fpu_status = grid_state.FPU_state[i].state;
        if ((fpu_status == FPST_READY_FORWARD)
                || (fpu_status == FPST_READY_REVERSE))
        {
            if ( ! (grid_state.FPU_state[i].waveform_valid
                    && grid_state.FPU_state[i].waveform_ready))
            {
                LOG_CONTROL(LOG_ERROR, "%18.6f : executeMotion(): error DE_WAVEFORM_NOT_READY for FPU %i: no waveform ready\n",
                            canlayer::get_realtime(), i);
                return DE_WAVEFORM_NOT_READY;
            }

            num_moving++;
        }
    }


    if (num_moving == 0)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : executeMotion(): error DE_NO_MOVABLE_FPUS: no FPUs present which can move\n",
                    canlayer::get_realtime());
        return DE_NO_MOVABLE_FPUS;
    }

    // acquire real-time priority so that consecutive broadcasts to
    // the different gateways are really sent in the same few
    // milliseconds.  (A lag of more than 10 - 20 milliseconds, for
    // example caused by low memory conditions and swapping, could
    // otherwise lead to collisions.)
    if (USE_REALTIME_SCHEDULING)
    {
        set_rt_priority(CONTROL_PRIORITY);
    }

    // FIXME: This is preliminary for use in the verificaiton
    // system. In the ICS driver, this needs to be changed to use the
    // gateway SYNC message to make sure that FPUs move with minimum
    // lag in respect to each other.

    E_DriverErrCode ecode = DE_OK;

    if (use_broadcast)
    {
        // send broadcast command to each gateway to start movement of all
        // FPUs.
        ecode = gateway.broadcastMessage<ExecuteMotionCommand>();
    }
    else
    {
        // send individual commands to FPUs which are not masked out
        unique_ptr<ExecuteMotionCommand> can_command;

        for (int i=0; i < config.num_fpus; i++)
        {
            if (fpuset[i])
            {
                can_command = gateway.provideInstance<ExecuteMotionCommand>();

                can_command->parametrize(i, use_broadcast);
                unique_ptr<I_CAN_Command> cmd(can_command.release());
                gateway.sendCommand(i, cmd);
            }
        }
    }

    // Give up real-time priority (this is important when the caller
    // thread later enters, for example, an endless loop).
    if (USE_REALTIME_SCHEDULING)
    {
        unset_rt_priority();
    }

    logGridState(config.logLevel, grid_state);

    LOG_CONTROL(LOG_INFO, "%18.6f : executeMotion(): executeMotion command successsfully sent to grid\n",
                canlayer::get_realtime());

    // adjust frequency of log entries
    log_repeat_count = 0;
    return ecode;

}

// counts the number of FPUs which will move with the given fpuset mask
int AsyncDriver::countMoving(const t_grid_state &grid_state, t_fpuset const &fpuset) const
{
    int num_moving = (grid_state.Counts[FPST_MOVING]
                      + grid_state.count_pending
                      + grid_state.num_queued);

    // The grid_state Counts member elements can include FPUs which
    // are masked out and will not move.  These must not be counted.

    int ready_count = (grid_state.Counts[FPST_READY_FORWARD]
                       + grid_state.Counts[FPST_READY_REVERSE]);

    if (ready_count >0)
    {
        for(int i=0; i < config.num_fpus; i++)
        {
            const t_fpu_state &fpu = grid_state.FPU_state[i];
            if ((fpu.state == FPST_READY_FORWARD)
                    || (fpu.state == FPST_READY_REVERSE))
            {
                if (fpuset[i])
                {
                    num_moving ++;
                }
                ready_count--;
                if (ready_count == 0)
                {
                    break;
                }
            }

        }
    }


    return num_moving;
}


E_DriverErrCode AsyncDriver::waitExecuteMotionAsync(t_grid_state& grid_state,
        E_GridState& state_summary,
        double &max_wait_time, bool &finished, t_fpuset const &fpuset)
{
    // Get number of FPUs which are moving or will move

    const t_grid_state previous_grid_state = grid_state;

    int num_moving = countMoving(grid_state, fpuset);

    bool cancelled = false;

    const unsigned long old_count_timeout = grid_state.count_timeout;


    LOG_CONTROL(LOG_VERBOSE, "%18.6f : waitExecuteMotion() - waiting for movement to complete\n",
                canlayer::get_realtime());

    if ( (num_moving > 0)
            && (grid_state.driver_state == DS_CONNECTED))
    {


        // this waits for finishing all pending messages,
        // all movement commands and leaving the READY_* states.
        state_summary = gateway.waitForState(TGT_NO_MORE_MOVING,
                                             grid_state, max_wait_time, cancelled);


        // We need to include the "ready" counts too because they
        // might take a moment to pick up the command.
        num_moving = countMoving(grid_state, fpuset);
    }

    finished = (! cancelled) && (num_moving == 0);

    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : waitExecuteMotion(): error DE_NO_CONNECTION, driver is not connected\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }


    if ((grid_state.Counts[FPST_OBSTACLE_ERROR] > 0) || (grid_state.Counts[FPST_ABORTED] > 0))
    {
        for (int i=0; i < config.num_fpus; i++)
        {

            const t_fpu_state& fpu = grid_state.FPU_state[i];
            E_FPU_STATE fpu_status = fpu.state;

            if (fpu_status == FPST_OBSTACLE_ERROR)
            {
                if (fpu.beta_collision)
                {
                    LOG_CONTROL(LOG_ERROR, "%18.6f : waitExecuteMotion(): error: DE_NEW_COLLISION detected for FPU %i.\n",
                                canlayer::get_realtime(), i);
                    logGridState(config.logLevel, grid_state);
                    fsync(config.fd_controllog);

                    return DE_NEW_COLLISION;
                }
                else
                {
                    LOG_CONTROL(LOG_ERROR, "%18.6f : waitExecuteMotion(): error: DE_NEW_LIMIT_BREACH detected for FPU %i.\n",
                                canlayer::get_realtime(), i);
                    logGridState(config.logLevel, grid_state);
                    fsync(config.fd_controllog);

                    return DE_NEW_LIMIT_BREACH;
                }
            }

            // step timing errors cause an FPU to change to ABORTED
            // state. To avoid confusion, a more specific error code is
            // returned.
            if (fpu.step_timing_errcount != previous_grid_state.FPU_state[i].step_timing_errcount)
            {
                LOG_CONTROL(LOG_ERROR, "%18.6f : waitExecuteMotion(): error: DE_STEP_TIMING_ERROR detected for FPU %i.\n",
                            canlayer::get_realtime(), i);

                logGridState(config.logLevel, grid_state);
                fsync(config.fd_controllog);

                return DE_STEP_TIMING_ERROR;
            }


            if (fpu_status == FPST_ABORTED)
            {
                LOG_CONTROL(LOG_ERROR, "%18.6f : waitExecuteMotion(): error: FPST_ABORTED state"
                            " detected for FPU %i, movement was aborted.\n",
                            canlayer::get_realtime(), i);

                logGridState(config.logLevel, grid_state);
                fsync(config.fd_controllog);

                return DE_MOVEMENT_ABORTED;
            }

        }
    }

    // It is important to compare for inequality here, because
    // count_timeout is an unsigned value which can intentionally wrap
    // without causing undefined behaviour.
    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : waitExecuteMotion(): error: DE_CAN_COMMAND_TIMEOUT_ERROR.\n",
                    canlayer::get_realtime());

        logGridState(config.logLevel, grid_state);
        fsync(config.fd_controllog);

        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    if (finished)
    {
        logGridState(config.logLevel, grid_state);

        LOG_CONTROL(LOG_INFO, "%18.6f : executeMotion(): movement successfully finished OK\n",
                    canlayer::get_realtime());
    }
    else
    {
        if (p_repeat_log(log_repeat_count))
        {

            LOG_CONTROL(LOG_GRIDSTATE, "%18.6f : executeMotion(): waiting time exceeded,"
                        " movement still incomplete\n",
                        canlayer::get_realtime());

            if (config.logLevel >= LOG_VERBOSE)
            {
                logGridState(config.logLevel, grid_state);

            }
        }
    }
    fsync(config.fd_controllog);

    return DE_OK;
}


E_DriverErrCode AsyncDriver::getPositionsAsync(t_grid_state& grid_state,
        E_GridState& state_summary, t_fpuset const &fpuset)
{

    // first, get current state of the grid
    state_summary = gateway.getGridState(grid_state);
    const unsigned long old_count_timeout = grid_state.count_timeout;
    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : getPositions():  error DE_NO_CONNECTION, connection was lost\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }



    unique_ptr<GetStepsAlphaCommand> can_command1;
    int num_skipped=0;
    for (int i=0; i < config.num_fpus; i++)
    {
        // we exclude locked FPUs
        if (! gateway.isLocked(i) )
        {
            if (!fpuset[i])
            {
                num_skipped++;
                continue;
            }
            can_command1 = gateway.provideInstance<GetStepsAlphaCommand>();
            assert(can_command1);
            bool broadcast = false;
            can_command1->parametrize(i, broadcast);
            // send the command (the actual sending happens
            // in the TX thread in the background).
            CommandQueue::E_QueueState qstate;
            unique_ptr<I_CAN_Command> cmd1(can_command1.release());
            qstate = gateway.sendCommand(i, cmd1);
            assert(qstate == CommandQueue::QS_OK);

        }
    }

    // We do not expect the locked FPUs to respond.
    // FIXME: This needs to be documented and checked
    // with the firmware protocol.
    int num_pending = config.num_fpus - grid_state.Counts[FPST_LOCKED] - num_skipped;

    // fpus are now responding in parallel.
    //
    // As long as any fpus need to respond, wait for
    // them to finish.
    while ( (num_pending > 0) && ((grid_state.driver_state == DS_CONNECTED)))
    {
        // as we do not effect any change on the grid,
        // we need to wait for any response event,
        // and filter out whether we are actually ready.

        ///state_summary = gateway.waitForState(E_WaitTarget(TGT_TIMEOUT),
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        // get fresh count of pending fpus.
        // The reason we add the unsent command is that
        // the Tx thread might not have had opportunity
        // to send all the commands.
        num_pending = (grid_state.count_pending + grid_state.num_queued);


    }

    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : getPositions():  error DE_NO_CONNECTION, connection was lost\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }


    unique_ptr<GetStepsBetaCommand> can_command2;
    for (int i=0; i < config.num_fpus; i++)
    {
        // we exclude locked FPUs
        if (! gateway.isLocked(i) )
        {
            if (!fpuset[i])
            {
                // num_skipped was set above
                continue;
            }
            can_command2 = gateway.provideInstance<GetStepsBetaCommand>();
            assert(can_command2);
            bool broadcast = false;
            can_command2->parametrize(i, broadcast);
            // send the command (the actual sending happens
            // in the TX thread in the background).
            CommandQueue::E_QueueState qstate;
            unique_ptr<I_CAN_Command> cmd2(can_command2.release());
            qstate = gateway.sendCommand(i, cmd2);
            assert(qstate == CommandQueue::QS_OK);

        }
    }

    num_pending = config.num_fpus - grid_state.Counts[FPST_LOCKED] - num_skipped;

    // fpus are now responding in parallel.
    //
    // As long as any fpus need to respond, wait for
    // them to finish.
    while ( (num_pending > 0) && ((grid_state.driver_state == DS_CONNECTED)))
    {
        // as we do not effect any change on the grid,
        // we need to wait for any response event,
        // and filter out whether we are actually ready.
        //state_summary = gateway.waitForState(E_WaitTarget(TGT_TIMEOUT),

        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        // get fresh count of pending fpus.
        // The reason we add the unsent command is that
        // the Tx thread might not have had opportunity
        // to send all the commands.
        num_pending = (grid_state.count_pending
                       + grid_state.num_queued);

    }

    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : getPositions():  error DE_NO_CONNECTION, connection was lost\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }


    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : getPositions(): error: DE_CAN_COMMAND_TIMEOUT_ERROR.\n",
                    canlayer::get_realtime());
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    logGridState(config.logLevel, grid_state);

    LOG_CONTROL(LOG_INFO, "%18.6f : getPositions(): positions were retrieved successfully.\n",
                canlayer::get_realtime());

    return DE_OK;

}



E_DriverErrCode AsyncDriver::getCounterDeviationAsync(t_grid_state& grid_state,
        E_GridState& state_summary, t_fpuset const &fpuset)
{

    // first, get current state of the grid
    state_summary = gateway.getGridState(grid_state);
    const unsigned long old_count_timeout = grid_state.count_timeout;

    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : getCounterDeviations():  error DE_NO_CONNECTION, connection was lost\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }



    int num_skipped = 0;
    unique_ptr<GetErrorAlphaCommand> can_command1;
    for (int i=0; i < config.num_fpus; i++)
    {
        // we exclude locked FPUs
        if (! gateway.isLocked(i) )
        {
            if (!fpuset[i])
            {
                num_skipped++;
                continue;
            }
            can_command1 = gateway.provideInstance<GetErrorAlphaCommand>();
            assert(can_command1);
            bool broadcast = false;
            can_command1->parametrize(i, broadcast);
            // send the command (the actual sending happens
            // in the TX thread in the background).
            CommandQueue::E_QueueState qstate;
            unique_ptr<I_CAN_Command> cmd1(can_command1.release());
            qstate = gateway.sendCommand(i, cmd1);
            assert(qstate == CommandQueue::QS_OK);

        }
    }

    // We do not expect the locked FPUs to respond.
    // FIXME: This needs to be documented and checked
    // with the firmware protocol.
    int num_pending = config.num_fpus - grid_state.Counts[FPST_LOCKED] - num_skipped;

    // fpus are now responding in parallel.
    //
    // As long as any fpus need to respond, wait for
    // them to finish.
    while ( (num_pending > 0) && ((grid_state.driver_state == DS_CONNECTED)))
    {
        // as we do not effect any change on the grid,
        // we need to wait for any response event,
        // and filter out whether we are actually ready.

        ///state_summary = gateway.waitForState(E_WaitTarget(TGT_TIMEOUT),
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        // get fresh count of pending fpus.
        // The reason we add the unsent command is that
        // the Tx thread might not have had opportunity
        // to send all the commands.
        num_pending = (grid_state.count_pending + grid_state.num_queued);


    }

    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : getCounterDeviations():  error DE_NO_CONNECTION, connection was lost\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }

    unique_ptr<GetErrorBetaCommand> can_command2;
    for (int i=0; i < config.num_fpus; i++)
    {
        // we exclude locked FPUs
        if (! gateway.isLocked(i) )
        {
            if (! fpuset[i])
            {
                // num_skipped was set above
                continue;
            }
            can_command2 = gateway.provideInstance<GetErrorBetaCommand>();
            assert(can_command2);
            bool broadcast = false;
            can_command2->parametrize(i, broadcast);
            // send the command (the actual sending happens
            // in the TX thread in the background).
            CommandQueue::E_QueueState qstate;
            unique_ptr<I_CAN_Command> cmd2(can_command2.release());
            qstate = gateway.sendCommand(i, cmd2);
            assert(qstate == CommandQueue::QS_OK);
        }
    }

    num_pending = config.num_fpus - grid_state.Counts[FPST_LOCKED] - num_skipped;

    // fpus are now responding in parallel.
    //
    // As long as any fpus need to respond, wait for
    // them to finish.
    while ( (num_pending > 0) && ((grid_state.driver_state == DS_CONNECTED)))
    {
        // as we do not effect any change on the grid,
        // we need to wait for any response event,
        // and filter out whether we are actually ready.
        //state_summary = gateway.waitForState(E_WaitTarget(TGT_TIMEOUT),

        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        // get fresh count of pending fpus.
        // The reason we add the unsent command is that
        // the Tx thread might not have had opportunity
        // to send all the commands.
        num_pending = (grid_state.count_pending
                       + grid_state.num_queued);

    }

    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : getCounterDeviations():  error DE_NO_CONNECTION, connection was lost\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }


    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : getCounterDeviations():  error DE_CAN_COMMAND_TIMEOUT_ERROR\n",
                    canlayer::get_realtime());

        logGridState(config.logLevel, grid_state);

        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    logGridState(config.logLevel, grid_state);

    LOG_CONTROL(LOG_INFO, "%18.6f : getCounterDeviations(): retrieved counter deviations successfully\n",
                canlayer::get_realtime());
    return DE_OK;

}


E_DriverErrCode AsyncDriver::repeatMotionAsync(t_grid_state& grid_state,
        E_GridState& state_summary, t_fpuset const &fpuset)
{

    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);

    const unsigned long old_count_timeout = grid_state.count_timeout;

    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : repeatMotion():  error DE_NO_CONNECTION, connection was lost\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }

    // check no FPUs have ongoing collisions or are moving
    for (int i=0; i < config.num_fpus; i++)
    {
        E_FPU_STATE fpu_status = grid_state.FPU_state[i].state;
        if (fpu_status == FPST_OBSTACLE_ERROR)
        {
            logGridState(config.logLevel, grid_state);
            LOG_CONTROL(LOG_ERROR, "%18.6f : repeatMotion():  error DE_UNRESOLVED_COLLISION for FPU %i,"
                        " collision needs to be resolvedfirst\n",
                        canlayer::get_realtime(), i);
            return DE_UNRESOLVED_COLLISION;
        }
        if (fpu_status == FPST_ABORTED)
        {
            logGridState(config.logLevel, grid_state);
            LOG_CONTROL(LOG_ERROR, "%18.6f : repeatMotion():  error DE_IN_ABORTED_STATE for FPU %i,"
                        " aborted state needs to be resolved first\n",
                        canlayer::get_realtime(), i);
            return DE_IN_ABORTED_STATE;
        }
    }

    for (int i=0; i < config.num_fpus; i++)
    {
        E_FPU_STATE fpu_status = grid_state.FPU_state[i].state;
        if (fpu_status == FPST_MOVING)
        {
            logGridState(config.logLevel, grid_state);

            LOG_CONTROL(LOG_ERROR, "%18.6f : repeatMotion():  error DE_SRILL_BUSY, FPU %i is still moving\n",
                        canlayer::get_realtime(), i);
            return DE_STILL_BUSY;
        }
    }

    /* check some FPUs in READY_* or RESTING state have valid waveforms
       This check intends to make sure that even in protocol version 1,
       waveforms are not used when they have been involved
       in collision or abort. */
    int count_movable = 0;
    for (int i=0; i < config.num_fpus; i++)
    {
        t_fpu_state fpu = grid_state.FPU_state[i];
        if (((fpu.state == FPST_READY_FORWARD)
                || (fpu.state == FPST_READY_REVERSE)
                || (fpu.state == FPST_RESTING))
                && fpu.waveform_valid
                && fpuset[i])
        {
            count_movable++;
        }
    }
    if (count_movable == 0)
    {
        logGridState(config.logLevel, grid_state);

        LOG_CONTROL(LOG_ERROR, "%18.6f : repeatMotion():  error DE_NO_MOVABLE_FPUs, "
                    "no FPUs are eligible to move\n",
                    canlayer::get_realtime());
        return DE_NO_MOVABLE_FPUS;
    }


    // All fpus which are in RESTING or READY_FORWARD state get a repeatMotion message.

    int cnt_pending = 0;
    unique_ptr<RepeatMotionCommand> can_command;
    for (int i=0; i < config.num_fpus; i++)
    {
        t_fpu_state& fpu_state = grid_state.FPU_state[i];
        // we exclude moving FPUs, but include FPUs which are
        // searching datum. (FIXME: double-check that).
        if (((fpu_state.state == FPST_READY_FORWARD)
                || (fpu_state.state == FPST_RESTING))
                && fpu_state.waveform_valid
                && fpuset[i])
        {

            // We use a non-broadcast instance. The advantage of
            // this is that he CAN protocol is able to reliably
            // detect whether this command was received - for
            // a broadcast command, this is not absolutely sure.
            bool broadcast = false;
            can_command = gateway.provideInstance<RepeatMotionCommand>();

            can_command->parametrize(i, broadcast);
            unique_ptr<I_CAN_Command> cmd(can_command.release());
            gateway.sendCommand(i, cmd);
            cnt_pending++;

        }
    }

#if 0
    // in Protocol version 1, we need to send a ping
    // because reverseMotion and repeatMotion do not
    // get a response.
    return pingFPUsAsync(grid_state, state_summary);
#else

    // wait until all generated messages have been responded to
    // or have timed out.
    while ( (cnt_pending > 0) && ((grid_state.driver_state == DS_CONNECTED)))
    {
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        cnt_pending = (grid_state.count_pending
                       + grid_state.num_queued);
    }
#endif

    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : repeatMotion():  error DE_NO_CONNECTION, connection was lost\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }

    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : repeatMotion():  error DE_CAN_COMMAND_TIMEOUT_ERROR, connection was lost\n",
                    canlayer::get_realtime());

        logGridState(config.logLevel, grid_state);

        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    LOG_CONTROL(LOG_INFO, "%18.6f : repeatMotion(): command successfully sent OK\n",
                canlayer::get_realtime());

    logGridState(config.logLevel, grid_state);

    return DE_OK;

}


E_DriverErrCode AsyncDriver::reverseMotionAsync(t_grid_state& grid_state,
        E_GridState& state_summary, t_fpuset const &fpuset)
{

    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);

    const unsigned long old_count_timeout = grid_state.count_timeout;

    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : reverseMotion():  error DE_NO_CONNECTION, connection was lost\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }

    // check no FPUs have ongoing collisions or are moving
    for (int i=0; i < config.num_fpus; i++)
    {
        E_FPU_STATE fpu_status = grid_state.FPU_state[i].state;
        if (fpu_status == FPST_OBSTACLE_ERROR)
        {
            logGridState(config.logLevel, grid_state);

            LOG_CONTROL(LOG_ERROR, "%18.6f : reverseMotion():  error DE_UNRESOLVED_COLLISON for FPU %i\n",
                        canlayer::get_realtime(), i);
            return DE_UNRESOLVED_COLLISION;
        }
        if (fpu_status == FPST_ABORTED)
        {
            logGridState(config.logLevel, grid_state);

            LOG_CONTROL(LOG_ERROR, "%18.6f : reverseMotion():  error DE_IN_ABORTED_STATE for FPU %i\n",
                        canlayer::get_realtime(), i);
            return DE_IN_ABORTED_STATE;
        }
    }

    for (int i=0; i < config.num_fpus; i++)
    {
        E_FPU_STATE fpu_status = grid_state.FPU_state[i].state;
        if (fpu_status == FPST_MOVING)
        {
            logGridState(config.logLevel, grid_state);

            LOG_CONTROL(LOG_ERROR, "%18.6f : reverseMotion():  error DE_SRILL_BUSY, FPU %i is still moving\n",
                        canlayer::get_realtime(), i);
            return DE_STILL_BUSY;
        }
    }

    /* check some FPUs in READY_* or RESTING state have valid waveforms
       This check intends to make sure that even in protocol version 1,
       waveforms are not used when they have been involved
       in collision or abort. */
    int count_movable = 0;
    for (int i=0; i < config.num_fpus; i++)
    {
        t_fpu_state fpu = grid_state.FPU_state[i];
        if (((fpu.state == FPST_READY_FORWARD)
                || (fpu.state == FPST_READY_REVERSE)
                || (fpu.state == FPST_RESTING))
                && fpu.waveform_valid
                && fpuset[i])
        {
            count_movable++;
        }
    }
    if (count_movable == 0)
    {
        logGridState(config.logLevel, grid_state);

        LOG_CONTROL(LOG_ERROR, "%18.6f : reverseMotion():  error DE_NO_MOVABLE_FPUs, "
                    "no FPUs are eligible to move\n",
                    canlayer::get_realtime());
        return DE_NO_MOVABLE_FPUS;
    }


    // All fpus which are in RESTING or READY_FORWARD state get a reverseMotion message.

    int cnt_pending = 0;
    unique_ptr<ReverseMotionCommand> can_command;
    for (int i=0; i < config.num_fpus; i++)
    {
        t_fpu_state& fpu_state = grid_state.FPU_state[i];
        if (((fpu_state.state == FPST_READY_FORWARD)
                || (fpu_state.state == FPST_RESTING))
                && fpu_state.waveform_valid
                && fpuset[i])
        {

            // We use a non-broadcast instance. The advantage of
            // this is that he CAN protocol is able to reliably
            // detect whether this command was received - for
            // a broadcast command, this is not absolutely sure.
            bool broadcast = false;
            can_command = gateway.provideInstance<ReverseMotionCommand>();

            can_command->parametrize(i, broadcast);
            unique_ptr<I_CAN_Command> cmd(can_command.release());
            gateway.sendCommand(i, cmd);
            cnt_pending++;

        }
    }

#if 0
    // in Protocol version 1, we need to send a ping
    // because reverseMotion and repeatMotion do not
    // get a response.
    return pingFPUsAsync(grid_state, state_summary, fpuset);
#else


    // wait until all generated messages have been responded to
    // or have timed out.
    while ( (cnt_pending > 0) && ((grid_state.driver_state == DS_CONNECTED)))
    {
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        cnt_pending = (grid_state.count_pending
                       + grid_state.num_queued);
    }
#endif

    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : reverseMotion():  error DE_NO_CONNECTION, connection was lost\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }

    if (grid_state.count_timeout != old_count_timeout)
    {
        logGridState(config.logLevel, grid_state);

        LOG_CONTROL(LOG_ERROR, "%18.6f : reverseMotion():  error DE_CAN_COMMAND_TIMEOUT_ERROR, connection was lost\n",
                    canlayer::get_realtime());
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    logGridState(config.logLevel, grid_state);

    LOG_CONTROL(LOG_INFO, "%18.6f : reverseMotion: command successfully sent OK\n",
                canlayer::get_realtime());

    return DE_OK;

}




E_DriverErrCode AsyncDriver::abortMotionAsync(pthread_mutex_t & command_mutex,
        t_grid_state& grid_state,
        E_GridState& state_summary,
        t_fpuset const &fpuset)
{


    // first, get current state of the grid
    state_summary = gateway.getGridState(grid_state);
    const unsigned long old_count_timeout = grid_state.count_timeout;

    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : abortMotion():  error DE_NO_CONNECTION, connection was lost\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }


    // acquire real-time priority so that consecutive broadcasts to
    // the different gateways are really sent in the same few
    // milliseconds.  (A lag of more than 10 - 20 milliseconds, for
    // example caused by low memory conditions and swapping, could
    // otherwise lead to collisions.)
    if (USE_REALTIME_SCHEDULING)
    {
        set_rt_priority(CONTROL_PRIORITY);
    }

    // this sends the abortMotion command directly.  It is implemented
    // as a gateway method so that lower layers have access to the
    // command when needed.


    // check whether we can use a fast broadcast command
    // this is the case if no FPU is masked out in
    // the fpuset parameter
    bool use_broadcast = true;
    for(int i=0; i < config.num_fpus; i++)
    {
        if (!fpuset[i])
        {
            use_broadcast = false;
            break;
        }
    }

    if (use_broadcast)
    {
        // send broadcast command
        gateway.abortMotion(grid_state, state_summary);
    }
    else
    {
        // send individual commands to FPUs which are not masked out
        unique_ptr<AbortMotionCommand> can_command;

        for (int i=0; i < config.num_fpus; i++)
        {
            if (fpuset[i])
            {
                can_command = gateway.provideInstance<AbortMotionCommand>();

                can_command->parametrize(i, use_broadcast);
                unique_ptr<I_CAN_Command> cmd(can_command.release());
                gateway.sendCommand(i, cmd);
            }
        }
    }

    // lock command mutex during waiting time for completion.
    pthread_mutex_lock(&command_mutex);


    // Give up real-time priority (this is important when the caller
    // thread later enters, for example, an endless loop).
    if (USE_REALTIME_SCHEDULING)
    {
        unset_rt_priority();
    }

    // Wait until all movements are cancelled.
    int num_moving = ( grid_state.Counts[FPST_MOVING]
                       + grid_state.Counts[FPST_DATUM_SEARCH]
                       + grid_state.count_pending
                       + grid_state.num_queued);

    while ( (num_moving > 0)
            && ((grid_state.driver_state == DS_CONNECTED)))
    {
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(TGT_NO_MORE_MOVING,
                                             grid_state, max_wait_time, cancelled);

        num_moving = (grid_state.Counts[FPST_MOVING]
                      + grid_state.Counts[FPST_DATUM_SEARCH]
                      + grid_state.count_pending
                      + grid_state.num_queued);
    }

    pthread_mutex_unlock(&command_mutex);

    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : abortMotion():  error DE_NO_CONNECTION, connection was lost\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }


    if (grid_state.count_timeout != old_count_timeout)
    {
        logGridState(config.logLevel, grid_state);

        LOG_CONTROL(LOG_ERROR, "%18.6f : abortMotion():  error DE_CAN_COMMAND_TIMEOUT_ERROR\n",
                    canlayer::get_realtime());
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    logGridState(config.logLevel, grid_state);

    LOG_CONTROL(LOG_INFO, "%18.6f : abortMotion(): command successfully sent\n",
                canlayer::get_realtime());
    return DE_OK;

}

#pragma GCC diagnostic push
#if CAN_PROTOCOL > 1
#pragma GCC diagnostic warning "-Wunused-parameter"
#else
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

E_DriverErrCode AsyncDriver::lockFPUAsync(int fpu_id, t_grid_state& grid_state,
        E_GridState& state_summary)
{
    // first, get current state of the grid
    state_summary = gateway.getGridState(grid_state);
    const unsigned long old_count_timeout = grid_state.count_timeout;

    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : lockFPU():  error DE_NO_CONNECTION, connection was lost\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }

    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : lockFPU():  error DE_CAN_COMMAND_TIMEOUT_ERROR\n",
                    canlayer::get_realtime());
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    logGridState(config.logLevel, grid_state);

    LOG_CONTROL(LOG_INFO, "%18.6f : lockFPU(): command successfully sent\n",
                canlayer::get_realtime());
    return DE_OK;
}
#pragma GCC diagnostic pop


#pragma GCC diagnostic push
#if CAN_PROTOCOL > 1
#pragma GCC diagnostic warning "-Wunused-parameter"
#else
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif
E_DriverErrCode AsyncDriver::unlockFPUAsync(int fpu_id, t_grid_state& grid_state,
        E_GridState& state_summary)
{
    // first, get current state of the grid
    state_summary = gateway.getGridState(grid_state);
    const unsigned long old_count_timeout = grid_state.count_timeout;

    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : unlockFPU():  error DE_NO_CONNECTION, connection was lost\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }

    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : unlockFPU():  error DE_CAN_COMMAND_TIMEOUT_ERROR\n",
                    canlayer::get_realtime());
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    logGridState(config.logLevel, grid_state);

    LOG_CONTROL(LOG_INFO, "%18.6f : unlockFPU(): command successfully sent\n",
                canlayer::get_realtime());
    return DE_OK;
}

#pragma GCC diagnostic pop

E_DriverErrCode AsyncDriver::pingFPUsAsync(t_grid_state& grid_state,
        E_GridState& state_summary, t_fpuset const &fpuset)
{

    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);
    const unsigned long old_count_timeout = grid_state.count_timeout;

    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : pingFPUs():  error DE_NO_CONNECTION, connection was lost\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }


    // All fpus which are not moving are pinged.

    int cnt_pending = 0;
    unique_ptr<PingFPUCommand> can_command;
    for (int i=0; i < config.num_fpus; i++)
    {
        t_fpu_state& fpu_state = grid_state.FPU_state[i];
        // we exclude moving FPUs, but include FPUs which are
        // searching datum.
        if ((fpu_state.state != FPST_MOVING) && fpuset[i])
        {

            // We use a non-broadcast instance. The advantage of
            // this is that he CAN protocol is able to reliably
            // detect whether this command was received - for
            // a broadcast command, this is not absolutely sure.
            bool broadcast = false;
            can_command = gateway.provideInstance<PingFPUCommand>();

            can_command->parametrize(i, broadcast);
            unique_ptr<I_CAN_Command> cmd(can_command.release());
            gateway.sendCommand(i, cmd);
            cnt_pending++;

        }
    }

    // wait until all generated ping commands have been responded to
    // or have timed out.
    while ( (cnt_pending > 0) && ((grid_state.driver_state == DS_CONNECTED)))
    {
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        if (grid_state.driver_state != DS_CONNECTED)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : pingFPUs():  error DE_NO_CONNECTION, connection was lost\n",
                        canlayer::get_realtime());
            return DE_NO_CONNECTION;
        }

        cnt_pending = (grid_state.count_pending
                       + grid_state.num_queued);
    }

    if (grid_state.count_timeout != old_count_timeout)
    {
        logGridState(config.logLevel, grid_state);

        LOG_CONTROL(LOG_ERROR, "%18.6f : pingFPUs():  error DE_CAN_COMMAND_TIMEOUT_ERROR\n",
                    canlayer::get_realtime());
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }


    LOG_CONTROL(LOG_INFO, "%18.6f : pingFPUs(): command successfully completed\n",
                canlayer::get_realtime());

    logGridState(config.logLevel, grid_state);
    return DE_OK;

}


E_DriverErrCode AsyncDriver::enableBetaCollisionProtectionAsync(t_grid_state& grid_state,
        E_GridState& state_summary)
{
    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);

    logGridState(config.logLevel, grid_state);

    const unsigned long old_count_timeout = grid_state.count_timeout;

    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : enableBetaCollisionProtection():  error DE_NO_CONNECTION, connection was lost\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }


    // make sure no FPU is moving or finding datum
    bool recoveryok=true;
    for (int i=0; i < config.num_fpus; i++)
    {
        t_fpu_state& fpu_state = grid_state.FPU_state[i];
        // we exclude moving FPUs and FPUs which are
        // searching datum.
        if ( (fpu_state.state == FPST_MOVING)
                && (fpu_state.state == FPST_DATUM_SEARCH))
        {
            recoveryok = false;
        }
    }

    if (! recoveryok)
    {
        // We do not allow recovery when there are moving FPUs.  (In
        // that case, the user should send an abortMotion command
        // first.)
        LOG_CONTROL(LOG_ERROR, "%18.6f : enableBetaCollisionProtection():  error DE_STILL_BUSY, "
                    "FPUs are still moving\n",
                    canlayer::get_realtime());
        return DE_STILL_BUSY;
    }


    unique_ptr<EnableBetaCollisionProtectionCommand> can_command;
    for (int i=0; i < config.num_fpus; i++)
    {
        bool broadcast = false;
        can_command = gateway.provideInstance<EnableBetaCollisionProtectionCommand>();
        can_command->parametrize(i, broadcast);
        unique_ptr<I_CAN_Command> cmd(can_command.release());
        gateway.sendCommand(i, cmd);
    }

    int cnt_pending = config.num_fpus;

    while ( (cnt_pending > 0) && ((grid_state.driver_state == DS_CONNECTED)))
    {
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        cnt_pending = (grid_state.count_pending
                       + grid_state.num_queued);
    }

    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : enableBetaCollisionProtection():  error DE_NO_CONNECTION, connection was lost\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }

    if (grid_state.count_timeout != old_count_timeout)
    {
        logGridState(config.logLevel, grid_state);

        LOG_CONTROL(LOG_ERROR, "%18.6f : enableBetaCollisionProtection():  error DE_CAN_COMMAND_TIMEOUT_ERROR\n",
                    canlayer::get_realtime());
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    logGridState(config.logLevel, grid_state);

    LOG_CONTROL(LOG_INFO, "%18.6f : enableBetaCollisionProtection(): command successfully sent to grid\n",
                canlayer::get_realtime());
    return DE_OK;
}

E_DriverErrCode AsyncDriver::freeBetaCollisionAsync(int fpu_id, E_REQUEST_DIRECTION request_dir,
        t_grid_state& grid_state,
        E_GridState& state_summary)
{
    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);

    const unsigned long old_count_timeout = grid_state.count_timeout;

    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : freeBetaCollision():  error DE_NO_CONNECTION, connection was lost\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }

    if ((fpu_id >= config.num_fpus) || (fpu_id < 0))
    {
        // the FPU id is out of range
        LOG_CONTROL(LOG_ERROR, "%18.6f : freeBetaCollision():  error DE_INVALID_FPU_ID, FPU id out of range\n",
                    canlayer::get_realtime());
        return DE_INVALID_FPU_ID;
    }


    // make sure no FPU is moving or finding datum
    bool recoveryok=true;
    for (int i=0; i < config.num_fpus; i++)
    {
        t_fpu_state& fpu_state = grid_state.FPU_state[i];
        // we exclude moving FPUs and FPUs which are
        // searching datum.
        if ( (fpu_state.state == FPST_MOVING)
                && (fpu_state.state == FPST_DATUM_SEARCH))
        {
            recoveryok = false;
        }
    }

    if (! recoveryok)
    {
        // We do not allow recovery when there are moving FPUs.  (In
        // that case, the user should send an abortMotion command
        // first.)

        logGridState(config.logLevel, grid_state);

        LOG_CONTROL(LOG_ERROR, "%18.6f : freeBetaCollision():  error DE_STILL_BUSY, "
                    "FPUs are still moving, if needed send abortMotion() first()\n",
                    canlayer::get_realtime());
        return DE_STILL_BUSY;
    }


    unique_ptr<FreeBetaCollisionCommand> can_command;
    can_command = gateway.provideInstance<FreeBetaCollisionCommand>();
    can_command->parametrize(fpu_id, request_dir);
    unique_ptr<I_CAN_Command> cmd(can_command.release());
    gateway.sendCommand(fpu_id, cmd);


    int cnt_pending = 1;

    while ( (cnt_pending > 0) && ((grid_state.driver_state == DS_CONNECTED)))
    {
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        cnt_pending = (grid_state.count_pending
                       + grid_state.num_queued);
    }

    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : freeBetaCollision():  error DE_NO_CONNECTION, connection was lost\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }

    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : freeBetaCollision():  error DE_CAN_COMMAND_TIMEOUT_ERROR\n",
                    canlayer::get_realtime());
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    logGridState(config.logLevel, grid_state);

    LOG_CONTROL(LOG_INFO, "%18.6f : freeBetaCollision(): command successfully sent to FPU %i\n",
                canlayer::get_realtime(), fpu_id);

    return DE_OK;
}


E_GridState AsyncDriver::getGridState(t_grid_state& out_state) const
{
    E_GridState state_summary = gateway.getGridState(out_state);
    logGridState(config.logLevel, out_state);
    return state_summary;
}

E_GridState AsyncDriver::waitForState(E_WaitTarget target,
                                      t_grid_state& out_detailed_state, double &max_wait_time, bool &cancelled) const
{
    return gateway.waitForState(target, out_detailed_state, max_wait_time, cancelled);
}



E_DriverErrCode AsyncDriver::setUStepLevelAsync(int ustep_level,
        t_grid_state& grid_state,
        E_GridState& state_summary,
        t_fpuset const &fpuset)
{
    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);

    const unsigned long old_count_timeout = grid_state.count_timeout;

    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : setUStepLevel():  error DE_NO_CONNECTION, connection was lost\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }

    switch (ustep_level)
    {
    case 1:
    case 2:
    case 4:
    case 8:
        break;
    default:
        // value is not allowed
        LOG_CONTROL(LOG_ERROR, "%18.6f : setUStepLevel():  error DE_INVALID_PAR_VALUE, level %i not allowed\n",
                    canlayer::get_realtime(), ustep_level);
        return DE_INVALID_PAR_VALUE;
    }


    for (int i=0; i < config.num_fpus; i++)
    {
        t_fpu_state& fpu_state = grid_state.FPU_state[i];
        // we exclude moving FPUs and FPUs which are
        // searching datum.
        if ( fpu_state.state != FPST_UNINITIALIZED)
        {
            // FPU state does not allows command
            LOG_CONTROL(LOG_ERROR, "%18.6f : setUStepLevel():  error DE_INVALID_FPU_STATE, all FPUs "
                        "need to be in state FPST_UNINITIALIZED\n",
                        canlayer::get_realtime());
            return DE_INVALID_FPU_STATE;
        }
    }


    int cnt_pending = 0;
    unique_ptr<SetUStepLevelCommand> can_command;
    for (int i=0; i < config.num_fpus; i++)
    {
        if (!fpuset[i])
        {
            continue;
        }
        // We use a non-broadcast instance. The advantage of
        // this is that the CAN protocol is able to reliably
        // detect whether this command was received - for
        // a broadcast command, this is not absolutely sure.
        bool broadcast = false;
        can_command = gateway.provideInstance<SetUStepLevelCommand>();

        can_command->parametrize(i, broadcast, ustep_level);
        unique_ptr<I_CAN_Command> cmd(can_command.release());
        gateway.sendCommand(i, cmd);
        cnt_pending++;
    }

    // wait until all generated commands have been responded to
    // or have timed out.
    while ( (cnt_pending > 0) && ((grid_state.driver_state == DS_CONNECTED)))
    {
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        if (grid_state.driver_state != DS_CONNECTED)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : setUStepLevel():  error DE_NO_CONNECTION, connection was lost\n",
                        canlayer::get_realtime());
            return DE_NO_CONNECTION;
        }

        cnt_pending = (grid_state.count_pending
                       + grid_state.num_queued);
    }

    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : setUStepLevel():  error DE_CAN_COMMAND_TIMEOUT_ERROR\n",
                    canlayer::get_realtime());

        logGridState(config.logLevel, grid_state);

        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    logGridState(config.logLevel, grid_state);

    LOG_CONTROL(LOG_INFO, "%18.6f : setUStepLevel(): command successfully sent, ustep_level set to %i\n",
                canlayer::get_realtime(), ustep_level);

    return DE_OK;

}

const char * str_driver_state(const E_DriverState driver_state)
{
    switch (driver_state)
    {
    case DS_UNINITIALIZED   :
        return "DS_UNINITIALIZED";
        break;
    case DS_UNCONNECTED:
        return "DS_UNCONNECTED";
        break;
    case DS_CONNECTED:
        return "DS_CONNECTED";
        break;
    case DS_ASSERTION_FAILED:
        return "DS_ASSERTION_FAILED";
        break;
    default:
        break;
    }
    return "undefined";
}

const char * str_fpu_state(const E_FPU_STATE state)
{

    switch (state)
    {
    case FPST_UNKNOWN       :
        return "UNKNOWN";
        break;
    case FPST_UNINITIALIZED :
        return "UNINITIALIZED";
        break;
    case FPST_LOCKED        :
        return "LOCKED";
        break;
    case FPST_DATUM_SEARCH  :
        return "DATUM_SEARCH";
        break;
    case FPST_AT_DATUM      :
        return "AT_DATUM";
        break;
    case FPST_LOADING       :
        return "LOADING";
        break;
    case FPST_READY_FORWARD :
        return "READY_FORWARD";
        break;
    case FPST_READY_REVERSE:
        return "READY_REVERSE";
        break;
    case FPST_MOVING        :
        return "MOVING";
        break;
    case FPST_RESTING       :
        return "RESTING";
        break;
    case FPST_ABORTED       :
        return "ABORTED";
        break;
    case FPST_OBSTACLE_ERROR:
        return "OBSTACLE_ERROR";
        break;
    default:
        break;
    }

    return "undefined";
}

void AsyncDriver::logGridState(const E_LogLevel logLevel, t_grid_state& grid_state) const
{
    char log_buffer[1024];
    char * logp = log_buffer;

    if (logLevel < LOG_INFO)
    {
        return;
    }

    double cur_time = canlayer::get_realtime();
    double t_offset;
    {
        // compute difference between monotonic time and
        // current UNIX time
        timespec mon_time_ref;
        get_monotonic_time(mon_time_ref);
        t_offset = cur_time - (mon_time_ref.tv_sec + 1e-9 * mon_time_ref.tv_nsec);
    }

    if (logLevel >= LOG_DEBUG)
    {
        logp += sprintf(logp, "driver state: DS=%s,  count_timeout=%lu, count_pending=%i, nqueued=%i",
                        str_driver_state(grid_state.driver_state),
                        grid_state.count_timeout,
                        grid_state.count_pending,
                        grid_state.num_queued);

        LOG_CONTROL(LOG_INFO, "%18.6f : %s\n",
                    cur_time,
                    log_buffer);
    }


    if (logLevel >= LOG_GRIDSTATE)
    {
        bool extra_verbose = ((grid_state.Counts[FPST_OBSTACLE_ERROR] > 0)
                              || (grid_state.Counts[FPST_ABORTED] > 0));

        for(int i=0; i < config.num_fpus; i++)
        {
            logp = log_buffer;
            const t_fpu_state &fpu = grid_state.FPU_state[i];

            double last_upd = t_offset + (fpu.last_updated.tv_sec + 1e-9 * fpu.last_updated.tv_nsec);

            logp += sprintf(logp, "FPU # %i: state=%-15.15s, steps = (%+6i, %+6i) = [%+9.3f, %+9.3f] deg, "
                            "az=%1u, bz=%1u, wvvalid=%1u, wvrdy=%1u, "
                            "lastcmd=%i, lastupd=%7.3f, tocount=%u",
                            i,
                            str_fpu_state(fpu.state),
                            fpu.alpha_steps,
                            fpu.beta_steps,
                            (fpu.alpha_steps / STEPS_PER_DEGREE_ALPHA) + config.alpha_datum_offset,
                            fpu.beta_steps / STEPS_PER_DEGREE_BETA,
                            fpu.alpha_was_zeroed,
                            fpu.beta_was_zeroed,
                            fpu.waveform_valid,
                            fpu.waveform_ready,
                            fpu.last_command,
                            last_upd,
                            fpu.timeout_count);


            if ((logLevel >= LOG_VERBOSE) || extra_verbose)
            {
                logp += sprintf(logp, ", pingok=%1u, mvcomplt=%1u, alimit=%1u, "
                                "collision=%1u, locked=%1u, npndng=%u, wvsegs=%3u, "
                                "wvrev=%1u",
                                fpu.ping_ok,
                                fpu.movement_complete,
                                fpu.at_alpha_limit,
                                fpu.beta_collision,
                                fpu.is_locked,
                                fpu.num_active_timeouts,
                                fpu.num_waveform_segments,
                                fpu.waveform_reversed
                               );

                if (logLevel >= LOG_DEBUG)
                {
                    logp += sprintf(logp, ", last_errcode=%i, prev_state=%s, pend_mask=0x%0x",
                                    fpu.last_status,
                                    str_fpu_state(fpu.previous_state),
                                    fpu.pending_command_set);
                }
            }



            LOG_CONTROL(LOG_GRIDSTATE, "%18.6f : %s \n",
                        cur_time,
                        log_buffer);

        }
    }

    logp = log_buffer;
    logp += sprintf(logp, "FPU state counts:");
    for (int i =0; i < NUM_FPU_STATES; i++)
    {
        logp += sprintf(logp, "\n\t\t\t%-15.15s\t: %4i,",
                        str_fpu_state(static_cast<E_FPU_STATE>(i)),
                        grid_state.Counts[i]);
    }
    LOG_CONTROL(LOG_INFO, "%18.6f : %s \n",
                cur_time,
                log_buffer);

}


E_DriverErrCode AsyncDriver::readRegisterAsync(uint16_t read_address,
        t_grid_state& grid_state,
        E_GridState& state_summary,
        t_fpuset const &fpuset)
{

    // first, get current state of the grid
    state_summary = gateway.getGridState(grid_state);
    const unsigned long old_count_timeout = grid_state.count_timeout;
    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : readRegister():  error DE_NO_CONNECTION, connection was lost\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }



    const uint8_t bank = (read_address >> 8) & 0xff;
    const uint8_t address_low_part = read_address & 0xff;
    unique_ptr<ReadRegisterCommand> can_command;
    int num_pending = 0;
    for (int i=0; i < config.num_fpus; i++)
    {
        // we exclude locked FPUs
        if ((! gateway.isLocked(i) ) && fpuset[i])
        {
            can_command = gateway.provideInstance<ReadRegisterCommand>();
            assert(can_command);
            bool broadcast = false;
            can_command->parametrize(i, broadcast, bank, address_low_part);
            // send the command (the actual sending happens
            // in the TX thread in the background).
            CommandQueue::E_QueueState qstate;
            unique_ptr<I_CAN_Command> cmd(can_command.release());
            qstate = gateway.sendCommand(i, cmd);
            assert(qstate == CommandQueue::QS_OK);
            num_pending++;

        }
    }


    // fpus are now responding in parallel.
    //
    // As long as any fpus need to respond, wait for
    // them to finish.
    while ( (num_pending > 0) && ((grid_state.driver_state == DS_CONNECTED)))
    {
        // as we do not effect any change on the grid,
        // we need to wait for any response event,
        // and filter out whether we are actually ready.

        ///state_summary = gateway.waitForState(E_WaitTarget(TGT_TIMEOUT),
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        // get fresh count of pending fpus.
        // The reason we add the unsent command is that
        // the Tx thread might not have had opportunity
        // to send all the commands.
        num_pending = (grid_state.count_pending + grid_state.num_queued);


    }

    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : readRegister():  error DE_NO_CONNECTION, connection was lost\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }

    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : readRegister(): error: DE_CAN_COMMAND_TIMEOUT_ERROR.\n",
                    canlayer::get_realtime());
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

#if (CAN_PROTOCOL_VERSION < 2)
    for(int i=0; i < config.num_fpus; i++)
    {
        // the version 2 response contains the address
        grid_state.FPU_state[i].register_address = read_address;
    }
#endif

    // log result if in debug mode
    if (config.logLevel >= LOG_DEBUG)
    {
        double log_time = canlayer::get_realtime();
        for(int i=0; i < config.num_fpus; i++)
        {
            LOG_CONTROL(LOG_INFO, "%18.6f : readregister: FPU # %4i, location %0x04u = %02xu.\n",
                        log_time, i, read_address, grid_state.FPU_state[i].register_value);

        }
    }

    LOG_CONTROL(LOG_INFO, "%18.6f : readRegister(): values were retrieved successfully.\n",
                canlayer::get_realtime());

    return DE_OK;

}

// get minimum firmware version value, using cache when valid
E_DriverErrCode AsyncDriver::getMinFirmwareVersion(t_fpuset const &fpuset,
        uint8_t (&min_firmware_version)[3],
        int &min_firmware_fpu,
        t_grid_state& grid_state,
        E_GridState& state_summary)
{

    min_firmware_fpu=-1;
    bool successfully_retrieved = false;

    // try to use cached value for FPU set
    getCachedMinFirmwareVersion(fpuset,
                                successfully_retrieved,
                                min_firmware_version,
                                min_firmware_fpu);

    if (! successfully_retrieved)
    {
        // we need to retrieve the firmware version first
        E_DriverErrCode ecode = getFirmwareVersionAsync(grid_state, state_summary, fpuset);
        if (ecode != DE_OK)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncDriver: findDatum(): "
                        "could not retrieve firmware versions - command cancelled\n",
                        canlayer::get_realtime());
            return ecode;
        }

        getCachedMinFirmwareVersion(fpuset,
                                    successfully_retrieved,
                                    min_firmware_version,
                                    min_firmware_fpu);

        if (! successfully_retrieved)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncDriver: findDatum(): "
                        "could not retrieve firmware versions - command cancelled\n",
                        canlayer::get_realtime());
            return ecode;
        }
    }
    return DE_OK;
}

void AsyncDriver::getCachedMinFirmwareVersion(t_fpuset const &fpuset,
        bool &was_retrieved,
        uint8_t (&min_firmware_version)[3],
        int &min_firmware_fpu) const
{

    min_firmware_fpu = 0;
    memset(min_firmware_version, FIRMWARE_NOT_RETRIEVED, sizeof(min_firmware_version));

    was_retrieved = false;

    for (int i=0; i < config.num_fpus; i++)
    {
        if (!fpuset[i])
        {
            continue;
        }

        bool is_smaller = false;
        for (int k=0; k <3; k++)
        {
            if (fpu_firmware_version[i][k] == FIRMWARE_NOT_RETRIEVED)
            {
                was_retrieved = false;
                memset(min_firmware_version, FIRMWARE_NOT_RETRIEVED, sizeof(min_firmware_version));
                return;
            }
            is_smaller = is_smaller || (min_firmware_version[k] >  fpu_firmware_version[i][k]);
        }

        if (is_smaller)
        {
            memcpy(min_firmware_version, fpu_firmware_version[i], sizeof(min_firmware_version));
            min_firmware_fpu = i;
            was_retrieved = true;
        }
    }

}

E_DriverErrCode AsyncDriver::getFirmwareVersionAsync(t_grid_state& grid_state,
        E_GridState& state_summary,
        t_fpuset const &fpuset)
{

    const int num_fields=6;
    uint8_t response_buffer[MAX_NUM_POSITIONERS][num_fields];
    memset(response_buffer, 0, sizeof(response_buffer));

    E_DriverErrCode ecode;

#if (CAN_PROTOCOL_VERSION > 1)
    return DE_FIRMWARE_UNIMPLEMENTED;
#endif


    for (int k=0; k < num_fields; k++)
    {
        ecode = readRegisterAsync(k, grid_state, state_summary, fpuset);
        if (ecode != DE_OK)
        {
            return ecode;
        }
        // copy data out of grid_state structure
        for (int i=0; i < config.num_fpus; i++)
        {
            if (! fpuset[i])
            {
                continue;
            }
            if (grid_state.FPU_state[i].register_address != k)
            {
                return DE_ASSERTION_FAILED;
            }
            response_buffer[i][k] = grid_state.FPU_state[i].register_value;
        }
    }


    // copy results back to grid state structure
    // (these results are *not* kept in the next call)
    for (int i=0; i < config.num_fpus; i++)
    {
        if (! fpuset[i])
        {
            continue;
        }

        for (int k=0; k < num_fields; k++)
        {
            if (k < 3)
            {
                grid_state.FPU_state[i].firmware_version[k] = response_buffer[i][k];
                fpu_firmware_version[i][k] = response_buffer[i][k];
            }
            else
            {
                grid_state.FPU_state[i].firmware_date[k-3] = response_buffer[i][k];
            }
        }
    }

    LOG_CONTROL(LOG_INFO, "%18.6f : getFirmwareVersion(): retrieved firmware versions successfully\n",
                canlayer::get_realtime());

    LOG_CONTROL(LOG_INFO, "%18.6f : getFirmwareVersion(): WARNING: result values are not kept\n",
                canlayer::get_realtime());
    return DE_OK;

}

E_DriverErrCode AsyncDriver::readSerialNumbersAsync(t_grid_state& grid_state,
        E_GridState& state_summary, t_fpuset const &fpuset)
{

    // first, get current state of the grid
    state_summary = gateway.getGridState(grid_state);
    const unsigned long old_count_timeout = grid_state.count_timeout;

    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : readSerialNumbers():  error DE_NO_CONNECTION, connection was lost\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }

    uint8_t min_firmware_version[3] = {0,0,0};
    int min_firmware_fpu = -1;

    E_DriverErrCode ecode = getMinFirmwareVersion(fpuset, min_firmware_version, min_firmware_fpu, grid_state, state_summary);

    if (ecode != DE_OK)
    {
        return ecode;
    }

    if ((min_firmware_version[0] < 1)
            ||(min_firmware_version[1] < 3))
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : readSerialNumbers():  error DE_FIRMWARE_UNIMPLEMENTED "
                    "- FPU firmware does not provide feature\n",
                    canlayer::get_realtime());
        return DE_FIRMWARE_UNIMPLEMENTED;
    }



    int num_skipped = 0;
    unique_ptr<ReadSerialNumberCommand> can_command;
    for (int i=0; i < config.num_fpus; i++)
    {
        if (!fpuset[i])
        {
            num_skipped++;
            continue;
        }
        can_command = gateway.provideInstance<ReadSerialNumberCommand>();
        assert(can_command);
        bool broadcast = false;
        can_command->parametrize(i, broadcast);
        // send the command (the actual sending happens
        // in the TX thread in the background).
        CommandQueue::E_QueueState qstate;
        unique_ptr<I_CAN_Command> cmd(can_command.release());
        qstate = gateway.sendCommand(i, cmd);
        assert(qstate == CommandQueue::QS_OK);

    }

    // We do not expect the locked FPUs to respond.
    // FIXME: This needs to be documented and checked
    // with the firmware protocol.
    int num_pending = config.num_fpus - grid_state.Counts[FPST_LOCKED] - num_skipped;

    // fpus are now responding in parallel.
    //
    // As long as any fpus need to respond, wait for
    // them to finish.
    while ( (num_pending > 0) && ((grid_state.driver_state == DS_CONNECTED)))
    {
        // as we do not effect any change on the grid,
        // we need to wait for any response event,
        // and filter out whether we are actually ready.

        ///state_summary = gateway.waitForState(E_WaitTarget(TGT_TIMEOUT),
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        // get fresh count of pending fpus.
        // The reason we add the unsent command is that
        // the Tx thread might not have had opportunity
        // to send all the commands.
        num_pending = (grid_state.count_pending + grid_state.num_queued);
    }

    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : readSerialNumbers():  error DE_NO_CONNECTION, connection was lost\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }


    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : readSerialNumbers():  error DE_CAN_COMMAND_TIMEOUT_ERROR\n",
                    canlayer::get_realtime());

        logGridState(config.logLevel, grid_state);

        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    logGridState(config.logLevel, grid_state);

    LOG_CONTROL(LOG_INFO, "%18.6f : readSerialNumbers(): retrieved serial numbers\n",
                canlayer::get_realtime());
    for(int i=0; i < config.num_fpus; i++)
    {
        const double t = canlayer::get_realtime();
        if (fpuset[i])
        {
            LOG_CONTROL(LOG_INFO, "%18.6f : FPU %i : SN = %s\n",
                        t, i, grid_state.FPU_state[i].serial_number);
        }
    }
    return DE_OK;

}


E_DriverErrCode AsyncDriver::writeSerialNumberAsync(int fpu_id, const char serial_number[],
        t_grid_state& grid_state,
        E_GridState& state_summary)
{
    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);

    const unsigned long old_count_timeout = grid_state.count_timeout;

    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : writeSerialNumber():  error DE_NO_CONNECTION, connection was lost\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }

    if ((fpu_id >= config.num_fpus) || (fpu_id < 0))
    {
        // the FPU id is out of range
        LOG_CONTROL(LOG_ERROR, "%18.6f : writeSerialNumber():  error DE_INVALID_FPU_ID, FPU id out of range\n",
                    canlayer::get_realtime());
        return DE_INVALID_FPU_ID;
    }

    const int sn_len = strnlen(serial_number, LEN_SERIAL_NUMBER);
    if (sn_len == LEN_SERIAL_NUMBER)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : writeSerialNumber():  error DE_INVALID_PAR_VALUE,"
                    " serial number is too long (length %i, only %i characters allowed)\n",
                    canlayer::get_realtime(), sn_len, LEN_SERIAL_NUMBER-1);

        return DE_INVALID_PAR_VALUE;
    }
    // check that we have ASCII printable chars only
    for(int i = 0; i < sn_len; i++)
    {
        const char ch = serial_number[i];
        if ((ch < 32) || (ch > 126))
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : writeSerialNumber():  error DE_INVALID_PAR_VALUE,"
                        " only ASCII printable characters allowed\n",
                        canlayer::get_realtime());
            return DE_INVALID_PAR_VALUE;
        }
    }

    t_fpuset fpuset;
    for(int i = 0; i < config.num_fpus; i++)
    {
        fpuset[i]=true;
    }
    // get movement state
    E_DriverErrCode ecode = pingFPUsAsync(grid_state, state_summary, fpuset);

    if (ecode != DE_OK)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : pingFPUs failed, aborting writeSerialNumber() command \n",
                    canlayer::get_realtime());
        return ecode;
    }

    uint8_t min_firmware_version[3] = {0,0,0};
    int min_firmware_fpu = -1;
    ecode = getMinFirmwareVersion(fpuset, min_firmware_version, min_firmware_fpu, grid_state, state_summary);

    if ((min_firmware_version[0] < 1)
            ||(min_firmware_version[1] < 3))
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : writeSerialNumber():  error DE_FIRMWARE_UNIMPLEMENTED "
                    "- FPU firmware does not provide feature\n",
                    canlayer::get_realtime());
        return DE_FIRMWARE_UNIMPLEMENTED;
    }


    // get all existing numbers
    ecode = readSerialNumbersAsync(grid_state, state_summary, fpuset);

    if (ecode != DE_OK)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : readSerialNumbers failed, aborting writeSerialNumber() command \n",
                    canlayer::get_realtime());
        return ecode;
    }

    // make sure no FPU is moving or finding datum

    if ((grid_state.Counts[FPST_MOVING] > 0)
            || (grid_state.Counts[FPST_DATUM_SEARCH] > 0))
    {
        // We do not allow writing the serial number when there are
        // moving FPUs, because it can take a long time.

        logGridState(config.logLevel, grid_state);

        LOG_CONTROL(LOG_ERROR, "%18.6f : writeSerialNumber():  error DE_STILL_BUSY, "
                    "FPUs are moving, won't write serial number\n",
                    canlayer::get_realtime());
        return DE_STILL_BUSY;
    }

    // make sure no other FPU in the grid has a serial number equal to
    // the one we are flashing
    for (int i=0; i < config.num_fpus; i++)
    {
        if (i == fpu_id)
        {
            // we allow writing the same number again to the same FPU
            continue;
        }
        if ( strncmp(grid_state.FPU_state[i].serial_number,
                     serial_number,
                     LEN_SERIAL_NUMBER) == 0)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : writeSerialNumber():  error DE_DUPLICATE_SERIAL_NUMBER, "
                        "Serial number is already used by another FPU in the grid\n",
                        canlayer::get_realtime());
            return DE_DUPLICATE_SERIAL_NUMBER;

        }
    }


    unique_ptr<WriteSerialNumberCommand> can_command;
    can_command = gateway.provideInstance<WriteSerialNumberCommand>();
    can_command->parametrize(fpu_id, serial_number);
    unique_ptr<I_CAN_Command> cmd(can_command.release());
    gateway.sendCommand(fpu_id, cmd);


    int cnt_pending = 1;

    while ( (cnt_pending > 0) && ((grid_state.driver_state == DS_CONNECTED)))
    {
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        cnt_pending = (grid_state.count_pending + grid_state.num_queued);
    }

    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : writeSerialNumber():  error DE_NO_CONNECTION, connection was lost\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }

    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : writeSerialNumber():  error DE_CAN_COMMAND_TIMEOUT_ERROR\n",
                    canlayer::get_realtime());
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    logGridState(config.logLevel, grid_state);

    LOG_CONTROL(LOG_INFO, "%18.6f : writeSerialNumber(): FPU %i: serial number '%s' successfully written to FPU\n",
                canlayer::get_realtime(), fpu_id, serial_number);

    return DE_OK;
}


}

} // end of namespace
