// -*- mode: c++ -*-
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 UKRI. See file "LICENSE" for license information.
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// jnix      2017-10-18  Created interface class using Pablo Guiterrez' CAN client sample
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME AsyncInterface.h
//
// This class implements the low-level CAN interface for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////
#include <cassert>
#include <unistd.h>
#include <string.h>

#ifdef DEBUG
#include <stdio.h>
#endif

#include "ethercan/AsyncInterface.h"
#include "ethercan/GatewayInterface.h"
#include "ethercan/time_utils.h"

// we need to include the individual CAN commands
// as they are parametrized here.
// -- alphabetically sorted below --
#include "ethercan/cancommandsv2/AbortMotionCommand.h"
#include "ethercan/cancommandsv2/CheckIntegrityCommand.h"
#include "ethercan/cancommandsv2/ConfigureMotionCommand.h"
#include "ethercan/cancommandsv2/EnableAlphaLimitProtectionCommand.h"
#include "ethercan/cancommandsv2/EnableBetaCollisionProtectionCommand.h"
#include "ethercan/cancommandsv2/EnableMoveCommand.h"
#include "ethercan/cancommandsv2/ExecuteMotionCommand.h"
#include "ethercan/cancommandsv2/FindDatumCommand.h"
#include "ethercan/cancommandsv2/FreeAlphaLimitBreachCommand.h"
#include "ethercan/cancommandsv2/FreeBetaCollisionCommand.h"
#include "ethercan/cancommandsv2/GetFirmwareVersionCommand.h"
#include "ethercan/cancommandsv2/LockUnitCommand.h"
#include "ethercan/cancommandsv2/PingFPUCommand.h"
#include "ethercan/cancommandsv2/ReadFirmwareVersionCommand.h"
#include "ethercan/cancommandsv2/ReadRegisterCommand.h"
#include "ethercan/cancommandsv2/ReadSerialNumberCommand.h"
#include "ethercan/cancommandsv2/RepeatMotionCommand.h"
#include "ethercan/cancommandsv2/ResetFPUCommand.h"
#include "ethercan/cancommandsv2/ResetStepCounterCommand.h"
#include "ethercan/cancommandsv2/ReverseMotionCommand.h"
#include "ethercan/cancommandsv2/SetStepsPerSegmentCommand.h"
#include "ethercan/cancommandsv2/SetTicksPerSegmentCommand.h"
#include "ethercan/cancommandsv2/SetUStepLevelCommand.h"
#include "ethercan/cancommandsv2/UnlockUnitCommand.h"
#include "ethercan/cancommandsv2/WriteSerialNumberCommand.h"



namespace mpifps
{

namespace ethercanif
{

E_EtherCANErrCode AsyncInterface::initializeInterface()
{
    switch (gateway.getInterfaceState())
    {
    case DS_UNINITIALIZED:
        break;

    case DS_UNCONNECTED:
    case DS_CONNECTED:
        LOG_CONTROL(LOG_ERROR, "%18.6f : initializeInterface() - interface was already initialized\n",
                    ethercanif::get_realtime());
        return DE_INTERFACE_ALREADY_INITIALIZED;

    case DS_ASSERTION_FAILED:
    default:
        LOG_CONTROL(LOG_ERROR, "%18.6f : error during initializeInterface() - assertion failed\n",
                    ethercanif::get_realtime());
        return DE_ASSERTION_FAILED;
    }
    LOG_CONTROL(LOG_DEBUG, "%18.6f : initializing EtherCAN interface\n",
                ethercanif::get_realtime());
    return gateway.initialize();
}


E_EtherCANErrCode AsyncInterface::deInitializeInterface()
{
    switch (gateway.getInterfaceState())
    {
    case DS_ASSERTION_FAILED:
    case DS_UNCONNECTED:
        break;

    case DS_UNINITIALIZED:
        LOG_CONTROL(LOG_ERROR, "%18.6f : deinitializeInterface() - error: interface is already in uninitialized state \n",
                    ethercanif::get_realtime());
        return DE_INTERFACE_NOT_INITIALIZED;

    case DS_CONNECTED:
        LOG_CONTROL(LOG_ERROR, "%18.6f : deinitializeInterface() - error: can't deinitialize interface, it is still connected\n",
                    ethercanif::get_realtime());
        return DE_INTERFACE_STILL_CONNECTED;

    default:
        LOG_CONTROL(LOG_ERROR, "%18.6f : deinitializeInterface() - fatal error: assertion failed\n",
                    ethercanif::get_realtime());
        return DE_ASSERTION_FAILED;
    };

    LOG_CONTROL(LOG_INFO, "%18.6f : deinitializing interface\n",
                ethercanif::get_realtime());
    return gateway.deInitialize();
}

E_EtherCANErrCode AsyncInterface::connect(const int ngateways, const t_gateway_address gateway_addresses[])
{
    switch (gateway.getInterfaceState())
    {
    case DS_UNINITIALIZED:
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface::connect(): error: interface not initialized\n",
                    ethercanif::get_realtime());
        return DE_INTERFACE_NOT_INITIALIZED;

    case DS_UNCONNECTED:
        break;

    case DS_CONNECTED:
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface::connect(): error: interface already connected, needs to disconnect() first\n",
                    ethercanif::get_realtime());
        return DE_INTERFACE_ALREADY_CONNECTED;

    case DS_ASSERTION_FAILED:
    default:
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface::connect(): fatal error: assertion failed\n",
                    ethercanif::get_realtime());
        return DE_ASSERTION_FAILED;
    }

    // Make sure that the passed number of gateways can support the
    // configured number of FPUs.
    if (ngateways < (config.num_fpus + MAX_FPUS_PER_GATEWAY-1) / MAX_FPUS_PER_GATEWAY)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface::connect(): number of configured gateways is insufficient\n",
                    ethercanif::get_realtime());
        return DE_INSUFFICENT_NUM_GATEWAYS;
    }

    E_EtherCANErrCode err_code =  gateway.connect(ngateways, gateway_addresses);
    if (err_code == DE_OK)
    {
        num_gateways = ngateways;
    }
    LOG_CONTROL(LOG_INFO, "%18.6f : GridInterface::connect(): interface is connected to %i gateways\n",
                ethercanif::get_realtime(),
                num_gateways);

    return err_code;
}

E_EtherCANErrCode AsyncInterface::disconnect()
{

    switch (gateway.getInterfaceState())
    {
    case DS_UNINITIALIZED:
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface::disconnect(): error, interface not initialized\n",
                    ethercanif::get_realtime());
        return DE_INTERFACE_NOT_INITIALIZED;

    case DS_UNCONNECTED:
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface::disconnect(): error, interface not connected\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;

    case DS_CONNECTED:
    case DS_ASSERTION_FAILED:
    default:
        break;
    }

    LOG_CONTROL(LOG_DEBUG, "%18.6f : AsyncInterface::disconnect(): disconnecting interface\n",
                ethercanif::get_realtime());


    E_EtherCANErrCode err_code = gateway.disconnect();

    if (err_code == DE_OK)
    {
        num_gateways = 0;
        LOG_CONTROL(LOG_DEBUG, "%18.6f : disconnect(): OK\n",
                    ethercanif::get_realtime());

    }

    return err_code;

}


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

E_EtherCANErrCode AsyncInterface::initializeGridAsync(t_grid_state& grid_state,
        E_GridState& state_summary,
        t_fpuset const &fpuset)
{

    state_summary = GS_UNKNOWN;
    grid_state.interface_state = DS_ASSERTION_FAILED;

    LOG_CONTROL(LOG_INFO, "%18.6f : initializing grid\n",
                ethercanif::get_realtime());

    if (gateway.getInterfaceState() != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : initializeGridAsync() error: interface is not connected\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }

    LOG_CONTROL(LOG_INFO, "%18.6f : initializeGrid(): command successfully sent\n",
                ethercanif::get_realtime());

    return DE_OK;
}
#pragma GCC diagnostic pop

void AsyncInterface::getStateCount(const t_grid_state& grid_state, t_fpuset const * const pfpuset, t_counts &counts)
{

    if (pfpuset == nullptr)
    {
        // the state count is already in the grid_state struct
        memcpy(counts, grid_state.Counts, sizeof(counts));
        return;
    }

    memset(counts, 0, sizeof(counts));
    const t_fpuset& fpuset = *pfpuset;
    for (int i=0; i < config.num_fpus; i++)
    {
        if (fpuset[i])
        {
            counts[static_cast<int>(grid_state.FPU_state[i].state)]++;
        }
    }
}


E_EtherCANErrCode AsyncInterface::resetFPUsAsync(t_grid_state& grid_state,
        E_GridState& state_summary,
        t_fpuset const &fpuset,
        const bool include_locked_fpus)
{

    LOG_CONTROL(LOG_INFO, "%18.6f : resetting FPUs\n",
                ethercanif::get_realtime());

    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);
    const unsigned long old_count_timeout = grid_state.count_timeout;
    const unsigned long old_count_can_overflow = grid_state.count_can_overflow;
    // check interface is connected
    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : resetFPUs() error: interface is not connected, can't reset FPUs\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }

    t_counts scounts;
    getStateCount(grid_state, &fpuset, scounts);

    // make sure no FPU in the set is moving or finding datum
    bool resetok= ((scounts[FPST_MOVING] == 0) && (scounts[FPST_DATUM_SEARCH] == 0));

    if (! resetok)
    {
        // We do not perform a reset when there are moving FPUs.  (In
        // that case, the user should send an abortMotion command
        // first.)
        LOG_CONTROL(LOG_ERROR, "%18.6f : error: FPUs are moving, refusing to reset FPUs. Call abortMotion first.\n",
                    ethercanif::get_realtime());
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

        const t_fpu_state& fpu = grid_state.FPU_state[i];

        if ((fpu.state == FPST_LOCKED) && (! include_locked_fpus))
        {
            LOG_CONTROL(LOG_INFO, "%18.6f : skipping resetFPU(): FPU #%i is locked and will not be"
                        " reset (use include_locked_fpus flag to include it).\n",
                        ethercanif::get_realtime(), i);

            LOG_CONSOLE(LOG_INFO, "%18.6f : skipping resetFPU(): FPU #%i is locked and will not be"
                        " reset (use include_locked_fpus flag to include it).\n",
                        ethercanif::get_realtime(), i);
            continue;
        }

        bool broadcast = false;
        can_command = gateway.provideInstance<ResetFPUCommand>();
        can_command->parametrize(i, broadcast);
        unique_ptr<CAN_Command> cmd(can_command.release());
        gateway.sendCommand(i, cmd);
        cnt_pending++;
    }


    while ( (cnt_pending > 0) && ((grid_state.interface_state == DS_CONNECTED)))
    {
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        cnt_pending = (grid_state.count_pending + grid_state.num_queued);
    }


    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : error: interface is not connected, can't reset FPUs\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }

    // It is important to compare for inequality here, because
    // count_timeout is an unsigned value which can intentionally wrap
    // without causing undefined behaviour.
    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : resetFPUs():  error DE_CAN_COMMAND_TIMEOUT_ERROR\n",
                    ethercanif::get_realtime());
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    if (old_count_can_overflow != grid_state.count_can_overflow)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : resetFPUs():  error: firmware CAN buffer overflow\n",
                    ethercanif::get_realtime());

        return DE_FIRMWARE_CAN_BUFFER_OVERFLOW;
    }

    LOG_CONTROL(LOG_INFO, "%18.6f : resetFPUs: command completed succesfully\n",
                ethercanif::get_realtime());

    logGridState(config.logLevel, grid_state);

    return DE_OK;

}

void AsyncInterface::getFPUsetOpt(t_fpuset const * const fpuset_opt, t_fpuset &fpuset) const
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

E_EtherCANErrCode AsyncInterface::startAutoFindDatumAsync(t_grid_state& grid_state,
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
                        ethercanif::get_realtime());
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
                        ethercanif::get_realtime());
            return DE_INVALID_PAR_VALUE;
        }

        LOG_CONTROL(LOG_INFO, "%18.6f : AsyncInterface: findDatum started, arm_selection=%s, timeouts=%s\n",
                    ethercanif::get_realtime(), as_string, to_string);
    }

    bool contains_auto=false;
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
                        ethercanif::get_realtime(), direction_flags[i], i);
            return DE_INVALID_PAR_VALUE;
        }

        if (contains_auto && timeouts_disabled)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : findDatum(): error: time-outs disabled, but automatic search selected for FPU #%i\n",
                        ethercanif::get_realtime(), i);
            return DE_INVALID_PAR_VALUE;
        }
        LOG_CONTROL(LOG_INFO, "%18.6f : AsyncInterface: findDatum(): direction selection for FPU %i =%s\n",
                    ethercanif::get_realtime(), i, as_string);
    }


    E_EtherCANErrCode ecode = assureMinFirmwareVersion(2, 0, 0, "findDatum()", fpuset, grid_state);
    if (ecode != DE_OK)
    {
        return ecode;
    }

    // now, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);
    const unsigned long old_count_timeout = grid_state.count_timeout;
    const unsigned long old_count_can_overflow = grid_state.count_can_overflow;

    // check interface is connected
    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : findDatum(): error: interface is not connected\n",
                    ethercanif::get_realtime());
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
                    ethercanif::get_realtime(), int(arm_selection));
        return DE_INVALID_PAR_VALUE;
    }

    // In difference to protocol v1, the ping here is not needed any
    // more for position information.  However, we need to be sure we
    // have a working connection, because the datum command might
    // assume synchronous movement of multiple FPUs.
    ecode = pingFPUsAsync(grid_state, state_summary, fpuset);

    if (ecode != DE_OK)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : ping failed, aborting findDatum() operation \n",
                    ethercanif::get_realtime());
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
                        ethercanif::get_realtime(), i);
            return DE_UNRESOLVED_COLLISION;
        }
        if (fpu_status == FPST_ABORTED)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : FPU #%i is in aborted state - cancelling findDatum()operation.\n",
                        ethercanif::get_realtime(), i);
            return DE_IN_ABORTED_STATE;
        }

        if (fpu_status == FPST_MOVING)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : FPU #%i is in MOVING state - cancelling findDatum()operation.\n",
                        ethercanif::get_realtime(), i);
            return DE_INVALID_FPU_STATE;
        }

        if (fpu_status == FPST_DATUM_SEARCH)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : FPU #%i is in DATUM_SEARCH state - cancelling findDatum()operation.\n",
                        ethercanif::get_realtime(), i);
            return DE_INVALID_FPU_STATE;
        }


        if ( (fpu.alpha_datum_switch_active) && ((arm_selection == DASEL_ALPHA) || (arm_selection == DASEL_BOTH)))
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : FPU #%i has active alpha datum/limit switch"
                        " - cancelling findDatum()operation.\n",
                        ethercanif::get_realtime(), i);
            // We differentiate the error codes in one for the above pre-check, and
            // another for the firmware error message. This makes it possible
            // to derive the correct arm location in the protection layer.
            return DE_ALPHA_ARM_ON_LIMIT_SWITCH;
        }

        if (fpu_status == FPST_LOCKED)
        {
            LOG_CONTROL(LOG_INFO, "%18.6f : skipping findDAtum(): FPU #%i is locked and will not be moved.\n",
                        ethercanif::get_realtime(), i);
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
                            ethercanif::get_realtime(), i);
                return DE_PROTECTION_ERROR;
            }

            if (count_protection && (beta_steps > 0) && (beta_mode == SEARCH_ANTI_CLOCKWISE))
            {
                LOG_CONTROL(LOG_ERROR, "%18.6f : findDatum():"
                            " FPU %i: beta arm appears to be in positive position "
                            "and mode is SEARCH_ANTI_CLOCKWISE - aborting findDatum() operation \n",
                            ethercanif::get_realtime(), i);
                return DE_PROTECTION_ERROR;
            }

            if (count_protection && (! beta_initialized) && (beta_mode == SEARCH_AUTO))
            {
                LOG_CONTROL(LOG_ERROR, "%18.6f : findDatum():"
                            " FPU %i beta arm is uninitialized "
                            "and mode is SEARCH_AUTO - aborting findDatum() operation \n",
                            ethercanif::get_realtime(), i);
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
        if (( (1 << fpu_state.state) & ( (1 <<  FPST_UNINITIALIZED)
                                         | (1 << FPST_AT_DATUM)
                                         | (1 << FPST_LOADING)
                                         | (1 << FPST_READY_FORWARD)
                                         | (1 << FPST_READY_REVERSE)
                                         | (1 << FPST_RESTING))) != 0)
        {

            bool broadcast = false;
            can_command = gateway.provideInstance<FindDatumCommand>();

            can_command->parametrize(i, broadcast, direction_flags[i], arm_selection, timeout_flag);
            unique_ptr<CAN_Command> cmd(can_command.release());
            gateway.sendCommand(i, cmd);

        }
    }


    // It is important to compare for inequality here, because
    // count_timeout is an unsigned value which can intentionally wrap
    // without causing undefined behaviour.
    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : findDatum(): error: command timed out\n",
                    ethercanif::get_realtime());
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    if (old_count_can_overflow != grid_state.count_can_overflow)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : findDatum(): error: firmware CAN buffer overflow\n",
                    ethercanif::get_realtime());
        return DE_FIRMWARE_CAN_BUFFER_OVERFLOW;
    }


    LOG_CONTROL(LOG_INFO, "%18.6f : findDatum(): command successfully sent\n",
                ethercanif::get_realtime());

    log_repeat_count =0; // adjust frequency of logging
    return DE_OK;

}


// helper function which limits amount of logging during wait for
// command termination
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

E_EtherCANErrCode AsyncInterface::waitAutoFindDatumAsync(t_grid_state& grid_state,
        E_GridState& state_summary,
        double &max_wait_time, bool &finished, t_fpuset const * const fpuset_opt)
{
    t_fpuset fpuset;
    getFPUsetOpt(fpuset_opt, fpuset);

    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : waitFindDatum():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }


    bool cancelled = false;


    const unsigned long old_count_timeout = grid_state.count_timeout;
    const unsigned long old_count_can_overflow = grid_state.count_can_overflow;

    state_summary = gateway.waitForState(TGT_NO_MORE_MOVING,
                                         grid_state, max_wait_time, cancelled);

    int num_moving = (grid_state.Counts[FPST_DATUM_SEARCH]
                      + grid_state.count_pending
                      + grid_state.num_queued);

    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : waitFindDatum(): error: interface is not connected\n",
                    ethercanif::get_realtime());
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
                            ethercanif::get_realtime(), i);
                logGridState(config.logLevel, grid_state);
                fsync(config.fd_controllog);

                return DE_NEW_COLLISION;
            }
            else
            {
                LOG_CONTROL(LOG_ERROR, "%18.6f : waitFindDatum(): error: limit breach detected for FOU %i\n",
                            ethercanif::get_realtime(), i);
                logGridState(config.logLevel, grid_state);
                fsync(config.fd_controllog);

                return DE_NEW_LIMIT_BREACH;
            }
        }
        if (fpu_status == FPST_ABORTED)
        {

            if (fpu.last_status == MCE_ERR_DATUM_TIME_OUT)
            {
                LOG_CONTROL(LOG_ERROR, "%18.6f : waitFindDatum(): CRITICAL ERROR: Datum operation timed out for FPU %i\n",
                            ethercanif::get_realtime(), i);
                logGridState(config.logLevel, grid_state);
                fsync(config.fd_controllog);

                return DE_DATUM_COMMAND_HW_TIMEOUT;
            }
            else
            {
                LOG_CONTROL(LOG_ERROR, "%18.6f : waitFindDatum(): error: FPU movement was aborted for FPU %i\n",
                            ethercanif::get_realtime(), i);
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

        if ((fpu_status == FPST_UNINITIALIZED) && (fpu.last_status == MCE_ERR_DATUM_ON_LIMIT_SWITCH))
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : waitFindDatum(): error: FPU %i alpha arm on datum switch, movement rejected\n",
                        ethercanif::get_realtime(), i);
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
                    ethercanif::get_realtime());

        logGridState(config.logLevel, grid_state);
        fsync(config.fd_controllog);

        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    if (old_count_can_overflow != grid_state.count_can_overflow)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : waitFindDatum(): error: firmware CAN buffer overflow\n",
                    ethercanif::get_realtime());

        logGridState(config.logLevel, grid_state);
        fsync(config.fd_controllog);
        return DE_FIRMWARE_CAN_BUFFER_OVERFLOW;
    }


    finished = (num_moving == 0) && (! cancelled);


    for(int i=0; i < config.num_fpus; i++)
    {
        if (! fpuset[i])
        {
            continue;
        }
        if ((grid_state.FPU_state[i].state == FPST_UNINITIALIZED)
                && (grid_state.FPU_state[i].last_status == MCE_ERR_AUTO_DATUM_UNINITIALIZED))
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : findDatum(): error: DE_PROTECTION_ERROR, FPU denied automatic datum search\n",
                        ethercanif::get_realtime());
            return DE_PROTECTION_ERROR;
        }
    }



    if (finished)
    {
        LOG_CONTROL(LOG_INFO, "%18.6f : AsyncInterface: findDatum finished successfully\n",
                    ethercanif::get_realtime());

        logGridState(config.logLevel, grid_state);
        fsync(config.fd_controllog);

        return DE_OK;
    }
    else
    {
        if (p_repeat_log(log_repeat_count))
        {
            LOG_CONTROL(LOG_GRIDSTATE, "%18.6f : AsyncInterface: findDatum not finished, waiting time elapsed\n",
                        ethercanif::get_realtime());
            if (config.logLevel >= LOG_VERBOSE)
            {
                logGridState(config.logLevel, grid_state);
            }
        }
        fsync(config.fd_controllog);

        return DE_WAIT_TIMEOUT;
    }


}

E_EtherCANErrCode AsyncInterface::validateWaveformsV1(const t_wtable& waveforms,
        const int MIN_STEPS,
        const int MAX_STEPS,
        const int MAX_START_STEPS,
        const unsigned int MAX_NUM_SECTIONS,
        const double MAX_INCREASE_FACTOR) const
{

    LOG_CONTROL(LOG_INFO, "%18.6f : AsyncInterface: validating waveforms (ruleset V1)\n",
                ethercanif::get_realtime());

    const int num_loading =  waveforms.size();
    const unsigned int num_steps = waveforms[0].steps.size();

    if (MIN_STEPS > MAX_STEPS)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: error DE_INVALID_CONFIG:"
                    "  minimum step number limit is larger than maximum limit\n",
                    ethercanif::get_realtime());
        return DE_INVALID_CONFIG;
    }
    if (MAX_START_STEPS > MAX_STEPS)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: error DE_INVALID_CONFIG:"
                    " upper limit of step count during start exceeds maximum step count\n",
                    ethercanif::get_realtime());
        return DE_INVALID_CONFIG;
    }
    if (MAX_START_STEPS <= MIN_STEPS)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: error DE_INVALID_CONFIG:"
                    " upper limit of step count during start is smaller than minimum value\n",
                    ethercanif::get_realtime());
        return DE_INVALID_CONFIG;
    }
    if (MAX_INCREASE_FACTOR < 1.0)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: error DE_INVALID_CONFIG:"
                    " relative growth factor is smaller than 1.\n",
                    ethercanif::get_realtime());
        return DE_INVALID_CONFIG;
    }


    if (num_steps > MAX_NUM_SECTIONS)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: error DE_INVALID_WAVEFORM_TOO_MANY_SECTIONS:"
                    "  waveform has too many steps (%i)\n",
                    ethercanif::get_realtime(), num_steps);
        return DE_INVALID_WAVEFORM_TOO_MANY_SECTIONS;
    }

    for (int fpu_index=0; fpu_index < num_loading; fpu_index++)
    {
        const int fpu_id = waveforms[fpu_index].fpu_id;
        if ((fpu_id >= config.num_fpus) || (fpu_id < 0))
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: waveform error DE_INVALID_FPU_ID:"
                        " FPU ID %i in waveform table is out of range\n",
                        ethercanif::get_realtime(), fpu_id);
            return DE_INVALID_FPU_ID;
        }

        // require same number of steps for all FPUs
        if (waveforms[fpu_index].steps.size() != num_steps)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: error DE_INVALID_WAVEFORM_RAGGED:"
                        " waveforms for FPU %i have unequal length\n",
                        ethercanif::get_realtime(), fpu_id);
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
                    LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: error DE_INVALID_WAVEFORM_STEPCOUNT_TOO_LARGE:"
                                "fpu %i, %s arm, movement interval %i: step count exceeds maximum\n\n",
                                ethercanif::get_realtime(),
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
                    LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: DE_INVALID_WAVEFORM_CHANGE: "
                                "fpu %i, %s arm, movement interval %i: invalid step count change\n",
                                ethercanif::get_realtime(),
                                fpu_id, chan_idx == 0 ? "alpha" : "beta", sidx);
                    return DE_INVALID_WAVEFORM_CHANGE;
                }

                xa_last = xa;
                x_last_sign = x_sign;
            }
            if (xa_last > MAX_START_STEPS)
            {
                // last step count must be minimum or smaller
                LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: DE_INVALID_WAVEFORM_TAIL: "
                            "fpu %i, %s arm, movement interval %i: last step count too large\n",
                            ethercanif::get_realtime(),
                            fpu_id, chan_idx == 0 ? "alpha" : "beta", num_steps -1);
                return DE_INVALID_WAVEFORM_TAIL;
            }
        }
    }
    LOG_CONTROL(LOG_INFO, "%18.6f : AsyncInterface::validateWaveforms() waveform OK\n",
                ethercanif::get_realtime());

    return DE_OK;
}

E_EtherCANErrCode AsyncInterface::validateWaveformsV2(const t_wtable& waveforms,
        const int MIN_STEPS,
        const int MAX_STEPS,
        const int MAX_START_STEPS,
        const unsigned int MAX_NUM_SECTIONS,
        const double MAX_INCREASE_FACTOR) const
{

    LOG_CONTROL(LOG_INFO, "%18.6f : AsyncInterface: validating waveforms (ruleset V2)\n",
                ethercanif::get_realtime());

    const int num_loading =  waveforms.size();
    const unsigned int num_steps = waveforms[0].steps.size();

    if (MIN_STEPS > MAX_STEPS)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: error DE_INVALID_CONFIG:"
                    "  minimum step number limit is larger than maximum limit\n",
                    ethercanif::get_realtime());
        return DE_INVALID_CONFIG;
    }
    if (MAX_START_STEPS > MAX_STEPS)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: error DE_INVALID_CONFIG:"
                    " upper limit of step count during start exceeds maximum step count\n",
                    ethercanif::get_realtime());
        return DE_INVALID_CONFIG;
    }
    if (MAX_START_STEPS < MIN_STEPS)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: error DE_INVALID_CONFIG:"
                    " upper limit of step count during start is smaller than minimum value\n",
                    ethercanif::get_realtime());
        return DE_INVALID_CONFIG;
    }
    if (MAX_INCREASE_FACTOR < 1.0)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: error DE_INVALID_CONFIG:"
                    " relative growth factor is smaller than 1.\n",
                    ethercanif::get_realtime());
        return DE_INVALID_CONFIG;
    }


    if (num_steps > MAX_NUM_SECTIONS)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: error DE_INVALID_WAVEFORM_TOO_MANY_SECTIONS:"
                    "  waveform has too many steps (%i)\n",
                    ethercanif::get_realtime(), num_steps);
        return DE_INVALID_WAVEFORM_TOO_MANY_SECTIONS;
    }

    for (int fpu_index=0; fpu_index < num_loading; fpu_index++)
    {
        const int fpu_id = waveforms[fpu_index].fpu_id;
        if ((fpu_id >= config.num_fpus) || (fpu_id < 0))
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: waveform error DE_INVALID_FPU_ID:"
                        " FPU ID %i in waveform table is out of range\n",
                        ethercanif::get_realtime(), fpu_id);
            return DE_INVALID_FPU_ID;
        }

        // require same number of steps for all FPUs
        if (waveforms[fpu_index].steps.size() != num_steps)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: error DE_INVALID_WAVEFORM_RAGGED:"
                        " waveforms for FPU %i have unequal length\n",
                        ethercanif::get_realtime(), fpu_id);
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

                const bool is_last_step = (sidx == (num_steps -1));


                if (xa > MAX_STEPS)
                {
                    LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: error DE_INVALID_WAVEFORM_STEPCOUNT_TOO_LARGE:"
                                "fpu %i, %s arm, movement interval %i: step count exceeds maximum\n\n",
                                ethercanif::get_realtime(),
                                fpu_id, chan_idx == 0 ? "alpha" : "beta", sidx);
                    return DE_INVALID_WAVEFORM_STEPCOUNT_TOO_LARGE;
                }



                const int xa_small = std::min(xa_last, xa);
                const int xa_large = std::max(xa_last, xa);
                const int increase_limit = int(ceil(xa_small * MAX_INCREASE_FACTOR));

                const bool valid_acc = (
                                           // 1) movement into the same direction

                                           ((x_sign == x_last_sign)
                                            //   1a) and currently *stopping* to move and on last step of waveform
                                            && (( (xa < MIN_STEPS)
                                                  && is_last_step)
                                                // or, 1b) at least  MIN_STEPS and the larger
                                                // of both values not larger than the allowed
                                                // relative increase
                                                || ( (xa_small >= MIN_STEPS)
                                                     && (xa_large <= increase_limit))))

                                           // or, it is the last step and before was a zero count
                                           || ( (xa_last == 0)
                                                && ( (xa < MIN_STEPS)
                                                     && is_last_step))

                                           // or, has stopped to move, possibly from higher speed
                                           || ( (xa == 0)
                                                && (xa_last >= MIN_STEPS)
                                                && (xa_last <= MAX_START_STEPS))
                                           // or, a single entry with a small number of steps,
                                           // preceded by a pause
                                           || ( (xa <= MAX_START_STEPS)
                                                && (xa >= MIN_STEPS)
                                                && (xa_last == 0))
                                           // or, with or without a change of direction,
                                           // one step number zero and the other at least
                                           // MIN_STEPS - at start or end of a movement
                                           || ((xa_small == 0)
                                               && (xa_large >= MIN_STEPS)
                                               && (xa_large <= MAX_START_STEPS))
                                           // or, a pause in movement
                                           || ((xa_small == 0)
                                               && (xa_large == 0)));

                if (!valid_acc)
                {
                    LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: DE_INVALID_WAVEFORM_CHANGE: "
                                "fpu %i, %s arm, movement interval %i: invalid step count change\n",
                                ethercanif::get_realtime(),
                                fpu_id, chan_idx == 0 ? "alpha" : "beta", sidx);
                    return DE_INVALID_WAVEFORM_CHANGE;
                }

                xa_last = xa;
                x_last_sign = x_sign;
            }
            if (xa_last > MAX_START_STEPS)
            {
                // last step count must be minimum or smaller
                LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: DE_INVALID_WAVEFORM_TAIL: "
                            "fpu %i, %s arm, movement interval %i: last step count too large\n",
                            ethercanif::get_realtime(),
                            fpu_id, chan_idx == 0 ? "alpha" : "beta", num_steps -1);
                return DE_INVALID_WAVEFORM_TAIL;
            }
        }
    }
    LOG_CONTROL(LOG_INFO, "%18.6f : AsyncInterface::validateWaveforms() waveform OK\n",
                ethercanif::get_realtime());

    return DE_OK;
}


E_EtherCANErrCode AsyncInterface::validateWaveformsV3(const t_wtable& waveforms,
        const int MIN_STEPS,
        const int MAX_STEPS,
        const int MAX_START_STEPS,
        const unsigned int MAX_NUM_SECTIONS,
        const double MAX_INCREASE_FACTOR) const
{

    LOG_CONTROL(LOG_INFO, "%18.6f : AsyncInterface: validating waveforms (ruleset V2)\n",
                ethercanif::get_realtime());

    const int num_loading =  waveforms.size();
    const unsigned int num_steps = waveforms[0].steps.size();

    if (MIN_STEPS > MAX_STEPS)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: error DE_INVALID_CONFIG:"
                    "  minimum step number limit is larger than maximum limit\n",
                    ethercanif::get_realtime());
        return DE_INVALID_CONFIG;
    }
    if (MAX_START_STEPS > MAX_STEPS)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: error DE_INVALID_CONFIG:"
                    " upper limit of step count during start exceeds maximum step count\n",
                    ethercanif::get_realtime());
        return DE_INVALID_CONFIG;
    }
    if (MAX_START_STEPS < MIN_STEPS)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: error DE_INVALID_CONFIG:"
                    " upper limit of step count during start is smaller than minimum value\n",
                    ethercanif::get_realtime());
        return DE_INVALID_CONFIG;
    }
    if (MAX_INCREASE_FACTOR < 1.0)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: error DE_INVALID_CONFIG:"
                    " relative growth factor is smaller than 1.\n",
                    ethercanif::get_realtime());
        return DE_INVALID_CONFIG;
    }


    if (num_steps > MAX_NUM_SECTIONS)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: error DE_INVALID_WAVEFORM_TOO_MANY_SECTIONS:"
                    "  waveform has too many steps (%i)\n",
                    ethercanif::get_realtime(), num_steps);
        return DE_INVALID_WAVEFORM_TOO_MANY_SECTIONS;
    }

    for (int fpu_index=0; fpu_index < num_loading; fpu_index++)
    {
        const int fpu_id = waveforms[fpu_index].fpu_id;
        if ((fpu_id >= config.num_fpus) || (fpu_id < 0))
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: waveform error DE_INVALID_FPU_ID:"
                        " FPU ID %i in waveform table is out of range\n",
                        ethercanif::get_realtime(), fpu_id);
            return DE_INVALID_FPU_ID;
        }

        // require same number of steps for all FPUs
        if (waveforms[fpu_index].steps.size() != num_steps)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: error DE_INVALID_WAVEFORM_RAGGED:"
                        " waveforms for FPU %i have unequal length\n",
                        ethercanif::get_realtime(), fpu_id);
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

                const bool is_last_step = (sidx == (num_steps -1));


                if (xa > MAX_STEPS)
                {
                    LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: error DE_INVALID_WAVEFORM_STEPCOUNT_TOO_LARGE:"
                                "fpu %i, %s arm, movement interval %i: step count exceeds maximum\n\n",
                                ethercanif::get_realtime(),
                                fpu_id, chan_idx == 0 ? "alpha" : "beta", sidx);
                    return DE_INVALID_WAVEFORM_STEPCOUNT_TOO_LARGE;
                }



                const int xa_small = std::min(xa_last, xa);
                const int xa_large = std::max(xa_last, xa);
                const int increase_limit = int(ceil(xa_last * MAX_INCREASE_FACTOR));
                const int decrease_limit = int(floor(xa_last / (MAX_INCREASE_FACTOR * MAX_INCREASE_FACTOR)));

                const bool valid_acc = (
                                           // 1) movement into the same direction

                                           ((x_sign == x_last_sign)
                                            //   1a) and currently *stopping* to move and on last step of waveform
                                            && (( (xa < MIN_STEPS)
                                                  && is_last_step)
                                                // or, 1b) at least  MIN_STEPS and the larger
                                                // of both values not larger than the allowed
                                                // relative increase
                                                || ( (xa >= xa_last)
                                                     && (xa_last >= MIN_STEPS)
                                                     && (xa <= increase_limit))
                                                || ( (xa <= xa_last)
                                                     && (xa >= MIN_STEPS)
                                                     && (xa >= decrease_limit))))

                                           // or, it is the last step and before was a zero count
                                           || ( (xa_last == 0)
                                                && ( (xa < MIN_STEPS)
                                                     && is_last_step))

                                           // or, has stopped to move, from any speed
                                           || ( (xa == 0)
                                                && (xa_last >= MIN_STEPS))
                                           // or, a single entry with a small number of steps,
                                           // preceded by a pause
                                           || ( (xa <= MAX_START_STEPS)
                                                && (xa >= MIN_STEPS)
                                                && (xa_last == 0))
                                           // or, with or without a change of direction,
                                           // one step number zero and the other at least
                                           // MIN_STEPS - at start or end of a movement
                                           || ((xa_small == 0)
                                               && (xa_large >= MIN_STEPS)
                                               && (xa_large <= MAX_START_STEPS))
                                           // or, a pause in movement
                                           || ((xa_small == 0)
                                               && (xa_large == 0)));

                if (!valid_acc)
                {
                    LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: DE_INVALID_WAVEFORM_CHANGE: "
                                "fpu %i, %s arm, movement interval %i: invalid step count change\n",
                                ethercanif::get_realtime(),
                                fpu_id, chan_idx == 0 ? "alpha" : "beta", sidx);
                    return DE_INVALID_WAVEFORM_CHANGE;
                }

                xa_last = xa;
                x_last_sign = x_sign;
            }
            if (xa_last > MAX_START_STEPS)
            {
                // last step count must be minimum or smaller
                LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: DE_INVALID_WAVEFORM_TAIL: "
                            "fpu %i, %s arm, movement interval %i: last step count too large\n",
                            ethercanif::get_realtime(),
                            fpu_id, chan_idx == 0 ? "alpha" : "beta", num_steps -1);
                return DE_INVALID_WAVEFORM_TAIL;
            }
        }
    }
    LOG_CONTROL(LOG_INFO, "%18.6f : AsyncInterface::validateWaveforms() waveform OK\n",
                ethercanif::get_realtime());

    return DE_OK;
}


E_EtherCANErrCode AsyncInterface::validateWaveformsV4(const t_wtable& waveforms,
        const int MIN_STEPS,
        const int MAX_STEPS,
        const int MAX_START_STEPS,
        const unsigned int MAX_NUM_SECTIONS,
        const double MAX_INCREASE_FACTOR) const
{

    LOG_CONTROL(LOG_INFO, "%18.6f : AsyncInterface: validating waveforms (ruleset V4)\n",
                ethercanif::get_realtime());

    const int MAX_DCHANGE_STEPS = 120;
    const int num_loading =  waveforms.size();
    const unsigned int num_steps = waveforms[0].steps.size();

    if (MIN_STEPS > MAX_STEPS)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: error DE_INVALID_CONFIG:"
                    "  minimum step number limit is larger than maximum limit\n",
                    ethercanif::get_realtime());
        return DE_INVALID_CONFIG;
    }
    if (MAX_START_STEPS > MAX_STEPS)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: error DE_INVALID_CONFIG:"
                    " upper limit of step count during start exceeds maximum step count\n",
                    ethercanif::get_realtime());
        return DE_INVALID_CONFIG;
    }
    if (MAX_START_STEPS <= MIN_STEPS)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: error DE_INVALID_CONFIG:"
                    " upper limit of step count during start is smaller than minimum value\n",
                    ethercanif::get_realtime());
        return DE_INVALID_CONFIG;
    }
    if (MAX_INCREASE_FACTOR < 1.0)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: error DE_INVALID_CONFIG:"
                    " relative growth factor is smaller than 1.\n",
                    ethercanif::get_realtime());
        return DE_INVALID_CONFIG;
    }


    if (num_steps > MAX_NUM_SECTIONS)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: error DE_INVALID_WAVEFORM_TOO_MANY_SECTIONS:"
                    "  waveform has too many steps (%i)\n",
                    ethercanif::get_realtime(), num_steps);
        return DE_INVALID_WAVEFORM_TOO_MANY_SECTIONS;
    }

    for (int fpu_index=0; fpu_index < num_loading; fpu_index++)
    {
        const int fpu_id = waveforms[fpu_index].fpu_id;
        if ((fpu_id >= config.num_fpus) || (fpu_id < 0))
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: waveform error DE_INVALID_FPU_ID:"
                        " FPU ID %i in waveform table is out of range\n",
                        ethercanif::get_realtime(), fpu_id);
            return DE_INVALID_FPU_ID;
        }

        // require same number of steps for all FPUs
        if (waveforms[fpu_index].steps.size() != num_steps)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: error DE_INVALID_WAVEFORM_RAGGED:"
                        " waveforms for FPU %i have unequal length\n",
                        ethercanif::get_realtime(), fpu_id);
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
                    LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: error DE_INVALID_WAVEFORM_STEPCOUNT_TOO_LARGE:"
                                "fpu %i, %s arm, movement interval %i: step count exceeds maximum\n\n",
                                ethercanif::get_realtime(),
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
                                           // or, both current and last absolute step count
                                           // are below MAX_DCHANGE_STEPS, and can have
                                           // different sign (this would work for firmware 1.5.0)
                                           || ( (xa <= MAX_DCHANGE_STEPS)
                                                && (xa_last <= MAX_DCHANGE_STEPS))
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
                    LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: DE_INVALID_WAVEFORM_CHANGE: "
                                "fpu %i, %s arm, movement interval %i: invalid step count change\n",
                                ethercanif::get_realtime(),
                                fpu_id, chan_idx == 0 ? "alpha" : "beta", sidx);
                    return DE_INVALID_WAVEFORM_CHANGE;
                }

                xa_last = xa;
                x_last_sign = x_sign;
            }
            if (xa_last > MAX_START_STEPS)
            {
                // last step count must be minimum or smaller
                LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: DE_INVALID_WAVEFORM_TAIL: "
                            "fpu %i, %s arm, movement interval %i: last step count too large\n",
                            ethercanif::get_realtime(),
                            fpu_id, chan_idx == 0 ? "alpha" : "beta", num_steps -1);
                return DE_INVALID_WAVEFORM_TAIL;
            }
        }
    }
    LOG_CONTROL(LOG_INFO, "%18.6f : AsyncInterface::validateWaveforms() waveform OK\n",
                ethercanif::get_realtime());

    return DE_OK;
}


E_EtherCANErrCode AsyncInterface::configMotionAsync(t_grid_state& grid_state,
        E_GridState& state_summary,
        const t_wtable& waveforms,
        t_fpuset const &fpuset,
        bool allow_uninitialized,
        int ruleset_version)
{

    LOG_CONTROL(LOG_INFO, "%18.6f : AsyncInterface: calling configMotion()\n",
                ethercanif::get_realtime());

    // first, get current state of the grid
    state_summary = gateway.getGridState(grid_state);



    const int min_stepcount = int(floor(config.motor_minimum_frequency
                                        * WAVEFORM_SEGMENT_DURATION_MS  / 1000));

    // perform hardware protection checks unless
    // explicitly disabled.

    // check no FPUs have ongoing collisions
    // and all have been initialized
    for (int i=0; i < config.num_fpus; i++)
    {
        {
            E_FPU_STATE fpu_status = grid_state.FPU_state[i].state;
            if (fpu_status == FPST_OBSTACLE_ERROR)
            {
                LOG_CONTROL(LOG_ERROR, "%18.6f : configMotion(): error DE_UNRESOLVED_COLLISION"
                            " - unresolved collision active for FPU %i\n",
                            ethercanif::get_realtime(), i);
                return DE_UNRESOLVED_COLLISION;
            }
            // This wasn't enforced in protocol version 1,
            // because we do not have an enableMove command.
            // In protocol version 2, the user has to issue
            // enableMove first.
            if (fpu_status == FPST_ABORTED)
            {
                LOG_CONTROL(LOG_ERROR, "%18.6f : configMotion(): error DE_ABORTED_STATE"
                            " - FPU %i is in aborted state\n",
                            ethercanif::get_realtime(), i);
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
                            ethercanif::get_realtime(), i);
                return DE_FPUS_NOT_CALIBRATED;
            }

        }

    }

    {

        const int max_stepcount = int(ceil(config.motor_maximum_frequency
                                           * WAVEFORM_SEGMENT_DURATION_MS  / 1000));
        const int max_start_stepcount = int(ceil(config.motor_max_start_frequency
                                            * WAVEFORM_SEGMENT_DURATION_MS  / 1000));

        const double max_rel_increase = config.motor_max_rel_increase;

        E_EtherCANErrCode vwecode = DE_OK;

        switch (ruleset_version)
        {
        case 0:
            break;

        case 1:
            vwecode = validateWaveformsV1(waveforms,
                                          min_stepcount,
                                          max_stepcount,
                                          max_start_stepcount,
                                          ConfigureMotionCommand::MAX_NUM_SECTIONS,
                                          max_rel_increase);
            break;

        case 2:
            vwecode = validateWaveformsV2(waveforms,
                                          min_stepcount,
                                          max_stepcount,
                                          max_start_stepcount,
                                          ConfigureMotionCommand::MAX_NUM_SECTIONS,
                                          max_rel_increase);
            break;

        case 3:
            vwecode = validateWaveformsV3(waveforms,
                                          min_stepcount,
                                          max_stepcount,
                                          max_start_stepcount,
                                          ConfigureMotionCommand::MAX_NUM_SECTIONS,
                                          max_rel_increase);

            break;
        case 4:
            vwecode = validateWaveformsV4(waveforms,
                                          min_stepcount,
                                          max_stepcount,
                                          max_start_stepcount,
                                          ConfigureMotionCommand::MAX_NUM_SECTIONS,
                                          max_rel_increase);

            break;
        default:
            return DE_INVALID_PAR_VALUE;

        }


        if (vwecode != DE_OK)
        {
            // validity check failed
            return vwecode;
        }

    }


    // assure that interface is connected
    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : configMotion(): error DE_NO_CONNECTION - no connection present\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }

    unique_ptr<ConfigureMotionCommand> can_command;
    // loop over number of steps in the table
    const int num_steps = waveforms[0].steps.size();

    bool configured_fpus[MAX_NUM_POSITIONERS];
    memset(configured_fpus, 0, sizeof(configured_fpus));

    const int num_loading =  waveforms.size();
    const bool confirm_each_step = config.confirm_each_step;

    int step_index = 0;
    int resend_downcount = config.configmotion_max_resend_count;
    int alpha_cur[MAX_NUM_POSITIONERS];
    int beta_cur[MAX_NUM_POSITIONERS];

    const int confirmation_period = ((config.configmotion_confirmation_period <= 0)
                                     ? 1 :
                                     config.configmotion_confirmation_period);

    const unsigned long initial_count_timeout = grid_state.count_timeout;
    unsigned long old_count_timeout = grid_state.count_timeout;
    const unsigned long old_count_can_overflow = grid_state.count_can_overflow;

    while (step_index < num_steps)
    {
        const bool first_entry = (step_index == 0);
        const bool last_entry = (step_index == (num_steps-1));
        const bool request_confirmation = (first_entry
                                           || last_entry
                                           || confirm_each_step
                                           || ((step_index % confirmation_period) == 0));

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

            if ((fpu_index == 0)
                    && (step_index != 0)
                    && (config.waveform_upload_pause_us > 0))
            {
                // Wait a short time before talking to the same FPU again because the FPUs seem to be
                // in general a bit sluggish.
                // We don't care about signals here.
                usleep(config.waveform_upload_pause_us);
            }
            int fpu_id = waveforms[fpu_index].fpu_id;
            if ((fpu_id >= config.num_fpus) || (fpu_id < 0))
            {
                // the FPU id is out of range
                LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface::configMotion(): FPU id '%i' is out of range"
                            "needs to be between 0 and %i\n",
                            ethercanif::get_realtime(),
                            fpu_id, (config.num_fpus - 1));
                return DE_INVALID_FPU_ID;
            }

            if (! fpuset[fpu_id])
            {
                continue;
            }

            t_fpu_state& fpu_state = grid_state.FPU_state[fpu_id];
            if (fpu_state.state == FPST_LOCKED)
            {
                LOG_CONTROL(LOG_INFO, "%18.6f : configMotion(): FPU #%i is locked, skipping. Unlock first to move.\n",
                            ethercanif::get_realtime(), fpu_id);
                LOG_CONSOLE(LOG_INFO, "%18.6f : configMotion(): FPU #%i is locked, skipping. Unlock first to move.\n",
                            ethercanif::get_realtime(), fpu_id);
            }
            else
            {
                // get a command buffer
                can_command = gateway.provideInstance<ConfigureMotionCommand>();

                const t_step_pair& step = waveforms[fpu_index].steps[step_index];

                can_command->parametrize(fpu_id,
                                         step.alpha_steps,
                                         step.beta_steps,
                                         first_entry,
                                         last_entry,
                                         min_stepcount,
                                         request_confirmation);

                // send the command (the actual sending happens
                // in the TX thread in the background).
                unique_ptr<CAN_Command> cmd(can_command.release());
                alpha_cur[fpu_id] += step.alpha_steps;
                beta_cur[fpu_id] += step.beta_steps;

                LOG_CONTROL(LOG_VERBOSE, "%18.6f : configMotion(): sending wtable section %i, fpu # %i "
                            "= (%+4i, %+4i) steps --> pos (%7.3f, %7.3f) degree)\n",
                            ethercanif::get_realtime(),
                            step_index, fpu_id, step.alpha_steps, step.beta_steps,
                            (alpha_cur[fpu_id] / STEPS_PER_DEGREE_ALPHA) + config.alpha_datum_offset,
                            beta_cur[fpu_id] / STEPS_PER_DEGREE_BETA);

                gateway.sendCommand(fpu_id, cmd);
            }
        }

        /* Apparently, at least for some firmware version 1, we cannot
        send more than one configMotion command at a time,
        or else CAN commands will get lost. */
        if (request_confirmation)
        {
            /* Wait and check that all FPUs are registered in LOADING
               state.  This is needed to make sure we have later a clear
               state transition for finishing the load with the last
               flag set. */
            double max_wait_time = -1;
            bool cancelled = false;
            state_summary = gateway.waitForState(TGT_NO_MORE_PENDING,
                                                 grid_state, max_wait_time, cancelled);
            if (grid_state.interface_state != DS_CONNECTED)
            {
                LOG_CONTROL(LOG_ERROR, "%18.6f : configMotion(): error: interface is not connected\n",
                            ethercanif::get_realtime());


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

                const t_fpu_state& fpu_state = grid_state.FPU_state[fpu_id];
                // we retry if an FPU which we tried to configure and is
                // not locked did not change to FPST_LOADING state.

                if ((fpu_state.state != FPST_LOCKED)
                        && ( ((first_entry && (! last_entry))
                              &&  (fpu_state.state != FPST_LOADING))
                             || (last_entry
                                 &&  ((fpu_state.state != FPST_READY_FORWARD)
                                      || (fpu_state.num_waveform_segments != waveforms[fpu_index].steps.size() )))))
                {
                    if (resend_downcount <= 0)
                    {
                        return DE_MAX_RETRIES_EXCEEDED;
                    }
                    do_retry = true;
                    LOG_CONTROL(LOG_ERROR, "%18.6f : configMotion(): warning: "
                                "loading/ready state or number of waveform segments not confirmed for FPU #%i,"
                                " retry from start! (%i retries left)\n",
                                ethercanif::get_realtime(),
                                fpu_id,
                                resend_downcount);
                    LOG_CONSOLE(LOG_ERROR, "%18.6f : configMotion(): warning: "
                                "loading/ready state or number of waveform segments not confirmed for FPU #%i,"
                                " retry from start! (%i retries left)\n",
                                ethercanif::get_realtime(),
                                fpu_id,
                                resend_downcount);

                    LOG_CONSOLE(LOG_INFO, "%18.6f : configMotion(): warning: "
                                "loading/ready state not confirmed for FPU #%i,"
                                " retry from start! (%i retries left)\n",
                                ethercanif::get_realtime(),
                                fpu_id,
                                resend_downcount);
                }
            }
            if (do_retry)
            {
                // we start again with loading the first step
                // (re-sending data for all FPUs).
                step_index = 0;
                resend_downcount--;
                // squelch time-out error
                old_count_timeout = grid_state.count_timeout;
                continue;
            }

        }
        step_index++;
    }

    // FIXME: did not to work reliably with v1 firmware - check
    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : configMotion(): error: CAN command repeatedly timed out\n",
                    ethercanif::get_realtime());

        logGridState(config.logLevel, grid_state);

        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    if (grid_state.count_timeout != initial_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : configMotion(): error: CAN command had timed out, seems recovered by re-sending data\n",
                    ethercanif::get_realtime());

        LOG_CONSOLE(LOG_ERROR, "%18.6f : configMotion(): error: CAN command had timed out, seems recovered by re-sending data\n",
                    ethercanif::get_realtime());

        logGridState(config.logLevel, grid_state);
    }


    if (old_count_can_overflow != grid_state.count_can_overflow)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : configMotion(): error: firmware CAN buffer overflow\n",
                    ethercanif::get_realtime());

        logGridState(config.logLevel, grid_state);
        return DE_FIRMWARE_CAN_BUFFER_OVERFLOW;
    }



    for (int fpu_index=0; fpu_index < num_loading; fpu_index++)
    {
        int fpu_id = waveforms[fpu_index].fpu_id;

        if (! fpuset[fpu_id])
        {
            continue;
        }

        LOG_CONTROL(LOG_GRIDSTATE, "%18.6f : configMotion(): fpu # %i "
                    "--> pos (%5i, %5i) steps ~ (%+9.3f, %+9.3f) degree) - OK\n",
                    ethercanif::get_realtime(),
                    fpu_id,
                    alpha_cur[fpu_id],
                    beta_cur[fpu_id],
                    (alpha_cur[fpu_id] / STEPS_PER_DEGREE_ALPHA) + config.alpha_datum_offset,
                    beta_cur[fpu_id] / STEPS_PER_DEGREE_BETA);
    }

    LOG_CONTROL(LOG_INFO, "%18.6f : configMotion(): waveforms successfully sent OK\n",
                ethercanif::get_realtime());

    logGridState(config.logLevel, grid_state);

    return DE_OK;
}

#if 0
// TODO: this method is currently implemented in Python, so that input
// can be passed through the software protection layer.
E_EtherCANErrCode AsyncInterface::configPathsAsync(t_grid_state& grid_state,
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

E_EtherCANErrCode AsyncInterface::startExecuteMotionAsync(t_grid_state& grid_state,
        E_GridState& state_summary,
        t_fpuset const &fpuset)
{

    LOG_CONTROL(LOG_VERBOSE, "%18.6f : AsyncInterface: starting executeMotion()\n",
                ethercanif::get_realtime());

    // first, get current state of the grid
    state_summary = gateway.getGridState(grid_state);
    // check interface is connected
    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : executeMotion(): error DE_NO_CONNECTION, interface is not connected\n",
                    ethercanif::get_realtime());
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
                        ethercanif::get_realtime(), i);
            return DE_UNRESOLVED_COLLISION;
        }
        if (fpu_status == FPST_ABORTED)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : executeMotion(): error DE_ABORTED_STATE in FPU %i, FPUs are in aborted state\n",
                        ethercanif::get_realtime(), i);
            return DE_IN_ABORTED_STATE;
        }

    }


    int num_moving = 0; // Number of FPUs which will move
    bool use_broadcast = true; // flag whether we can use a fast broadcast command

    /* check all FPUs in READY_* state have valid waveforms
       This check intends to make sure that
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
                            ethercanif::get_realtime(), i);
                return DE_WAVEFORM_NOT_READY;
            }

            num_moving++;
        }
    }


    if (num_moving == 0)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : executeMotion(): error DE_NO_MOVABLE_FPUS: no FPUs present which can move\n",
                    ethercanif::get_realtime());
        return DE_NO_MOVABLE_FPUS;
    }

    // Optionally, acquire real-time priority so that consecutive broadcasts to
    // the different gateways are really sent in the same few
    // milliseconds. This is not needed if the EtherCAN gateway
    // sync mechanism is used, which synchronises the executeMotion
    // broadcast command by wire. (It might be needed if we move only
    // a selection of FPUs, as we can't use broadcast in this case).
    if (USE_REALTIME_SCHEDULING)
    {
        set_rt_priority(CONTROL_PRIORITY);
    }

    // FIXME: This is preliminary for use in the verification
    // system. In the ICS interface, this needs to be changed to use the
    // gateway SYNC message to make sure that FPUs move with minimum
    // lag in respect to each other.

    E_EtherCANErrCode ecode = DE_OK;

    if (use_broadcast)
    {
        // send broadcast command to each gateway to start movement of all
        // FPUs. Locked FPUs of course need to ignore this command!
        ecode = gateway.broadcastMessage<ExecuteMotionCommand>();
    }
    else
    {
        // send individual commands to FPUs which are not masked out or locked
        unique_ptr<ExecuteMotionCommand> can_command;

        for (int i=0; i < config.num_fpus; i++)
        {
            if (fpuset[i])
            {
                if (grid_state.FPU_state[i].state == FPST_LOCKED)
                {
                    LOG_CONTROL(LOG_INFO, "%18.6f : executeMotion(): FPU %i is locked - skipped\n",
                                ethercanif::get_realtime(), i);
                }
                else
                {
                    can_command = gateway.provideInstance<ExecuteMotionCommand>();

                    can_command->parametrize(i, use_broadcast);
                    unique_ptr<CAN_Command> cmd(can_command.release());
                    gateway.sendCommand(i, cmd);
                }
            }
        }
    }

    // Give up real-time priority (this is important when the caller
    // thread later enters, for example, a buggy endless loop).
    if (USE_REALTIME_SCHEDULING)
    {
        unset_rt_priority();
    }

    logGridState(config.logLevel, grid_state);

    LOG_CONTROL(LOG_INFO, "%18.6f : executeMotion(): executeMotion command successsfully sent to grid\n",
                ethercanif::get_realtime());

    // adjust frequency of log entries
    log_repeat_count = 0;
    return ecode;

}

// counts the number of FPUs which are moving or will move with the given fpuset mask
int AsyncInterface::countMoving(const t_grid_state &grid_state, t_fpuset const &fpuset) const
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


E_EtherCANErrCode AsyncInterface::waitExecuteMotionAsync(t_grid_state& grid_state,
        E_GridState& state_summary,
        double &max_wait_time, bool &finished, t_fpuset const &fpuset)
{
    // Get number of FPUs which are moving or will move

    const t_grid_state previous_grid_state = grid_state;

    int num_moving = countMoving(grid_state, fpuset);

    bool cancelled = false;

    const unsigned long old_count_timeout = grid_state.count_timeout;
    const unsigned long old_count_can_overflow = grid_state.count_can_overflow;


    LOG_CONTROL(LOG_VERBOSE, "%18.6f : waitExecuteMotion() - waiting for movement to complete\n",
                ethercanif::get_realtime());

    if ( (num_moving > 0)
            && (grid_state.interface_state == DS_CONNECTED))
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

    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : waitExecuteMotion(): error DE_NO_CONNECTION, interface is not connected\n",
                    ethercanif::get_realtime());
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
                    // new refers to that this is an event, not a state. We know the collision
                    // is new because otherwise, the movement wouldn't have been allowed.
                    LOG_CONTROL(LOG_ERROR, "%18.6f : waitExecuteMotion(): error: DE_NEW_COLLISION detected for FPU %i.\n",
                                ethercanif::get_realtime(), i);
                    logGridState(config.logLevel, grid_state);
                    fsync(config.fd_controllog);

                    return DE_NEW_COLLISION;
                }
                else
                {
                    LOG_CONTROL(LOG_ERROR, "%18.6f : waitExecuteMotion(): error: DE_NEW_LIMIT_BREACH detected for FPU %i.\n",
                                ethercanif::get_realtime(), i);
                    logGridState(config.logLevel, grid_state);
                    fsync(config.fd_controllog);

                    return DE_NEW_LIMIT_BREACH;
                }
            }

            // step timing errors cause an FPU to change to ABORTED
            // state. To avoid confusion, a more specific error code is
            // returned, even if it is not reflected in the enumerated state.
            if (fpu.step_timing_errcount != previous_grid_state.FPU_state[i].step_timing_errcount)
            {
                LOG_CONTROL(LOG_ERROR, "%18.6f : waitExecuteMotion(): error: DE_STEP_TIMING_ERROR detected for FPU %i.\n",
                            ethercanif::get_realtime(), i);

                logGridState(config.logLevel, grid_state);
                fsync(config.fd_controllog);

                return DE_STEP_TIMING_ERROR;
            }


            if (fpu_status == FPST_ABORTED)
            {
                LOG_CONTROL(LOG_ERROR, "%18.6f : waitExecuteMotion(): error: FPST_ABORTED state"
                            " detected for FPU %i, movement was aborted.\n",
                            ethercanif::get_realtime(), i);

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
                    ethercanif::get_realtime());

        logGridState(config.logLevel, grid_state);
        fsync(config.fd_controllog);

        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    if (old_count_can_overflow != grid_state.count_can_overflow)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : waitExecuteMotion(): error: firmware CAN buffer overflow.\n",
                    ethercanif::get_realtime());

        logGridState(config.logLevel, grid_state);
        fsync(config.fd_controllog);
        return DE_FIRMWARE_CAN_BUFFER_OVERFLOW;
    }


    if (finished)
    {
        logGridState(config.logLevel, grid_state);

        LOG_CONTROL(LOG_INFO, "%18.6f : executeMotion(): movement successfully finished OK\n",
                    ethercanif::get_realtime());
    }
    else
    {
        if (p_repeat_log(log_repeat_count))
        {

            LOG_CONTROL(LOG_GRIDSTATE, "%18.6f : executeMotion(): waiting time exceeded,"
                        " movement still incomplete\n",
                        ethercanif::get_realtime());

            if (config.logLevel >= LOG_VERBOSE)
            {
                logGridState(config.logLevel, grid_state);

            }
        }
    }
    fsync(config.fd_controllog);

    return DE_OK;
}




E_EtherCANErrCode AsyncInterface::repeatMotionAsync(t_grid_state& grid_state,
        E_GridState& state_summary, t_fpuset const &fpuset)
{

    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);

    const unsigned long old_count_timeout = grid_state.count_timeout;
    const unsigned long old_count_can_overflow = grid_state.count_can_overflow;

    // check interface is connected
    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : repeatMotion():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
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
                        ethercanif::get_realtime(), i);
            return DE_UNRESOLVED_COLLISION;
        }
        if (fpu_status == FPST_ABORTED)
        {
            logGridState(config.logLevel, grid_state);
            LOG_CONTROL(LOG_ERROR, "%18.6f : repeatMotion():  error DE_IN_ABORTED_STATE for FPU %i,"
                        " aborted state needs to be resolved first\n",
                        ethercanif::get_realtime(), i);
            return DE_IN_ABORTED_STATE;
        }
    }

    for (int i=0; i < config.num_fpus; i++)
    {
        E_FPU_STATE fpu_status = grid_state.FPU_state[i].state;
        if (fpu_status == FPST_MOVING)
        {
            logGridState(config.logLevel, grid_state);

            LOG_CONTROL(LOG_ERROR, "%18.6f : repeatMotion():  error DE_STILL_BUSY, FPU %i is still moving\n",
                        ethercanif::get_realtime(), i);
            return DE_STILL_BUSY;
        }
    }

    /* check some FPUs in READY_* or RESTING state have valid waveforms
       This check intends to make sure that
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
                    ethercanif::get_realtime());
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
            unique_ptr<CAN_Command> cmd(can_command.release());
            gateway.sendCommand(i, cmd);
            cnt_pending++;

        }
    }


    // wait until all generated messages have been responded to
    // or have timed out.
    // Keep in mind this command does not starts a new movement -
    // it only re-prepares the waveform table in the FPUs.
    while ( (cnt_pending > 0) && ((grid_state.interface_state == DS_CONNECTED)))
    {
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        cnt_pending = (grid_state.count_pending
                       + grid_state.num_queued);
    }


    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : repeatMotion():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }

    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : repeatMotion():  error DE_CAN_COMMAND_TIMEOUT_ERROR, connection was lost\n",
                    ethercanif::get_realtime());

        logGridState(config.logLevel, grid_state);

        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    if (old_count_can_overflow != grid_state.count_can_overflow)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : repeatMotion():  error: firmware CAN buffer overflow\n",
                    ethercanif::get_realtime());

        logGridState(config.logLevel, grid_state);
        return DE_FIRMWARE_CAN_BUFFER_OVERFLOW;
    }



    LOG_CONTROL(LOG_INFO, "%18.6f : repeatMotion(): command successfully sent OK\n",
                ethercanif::get_realtime());

    logGridState(config.logLevel, grid_state);

    return DE_OK;

}


E_EtherCANErrCode AsyncInterface::reverseMotionAsync(t_grid_state& grid_state,
        E_GridState& state_summary, t_fpuset const &fpuset)
{

    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);

    const unsigned long old_count_timeout = grid_state.count_timeout;
    const unsigned long old_count_can_overflow = grid_state.count_can_overflow;

    // check interface is connected
    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : reverseMotion():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
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
                        ethercanif::get_realtime(), i);
            return DE_UNRESOLVED_COLLISION;
        }
        if (fpu_status == FPST_ABORTED)
        {
            logGridState(config.logLevel, grid_state);

            LOG_CONTROL(LOG_ERROR, "%18.6f : reverseMotion():  error DE_IN_ABORTED_STATE for FPU %i\n",
                        ethercanif::get_realtime(), i);
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
                        ethercanif::get_realtime(), i);
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
                    ethercanif::get_realtime());
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
            unique_ptr<CAN_Command> cmd(can_command.release());
            gateway.sendCommand(i, cmd);
            cnt_pending++;

        }
    }



    // wait until all generated messages have been responded to
    // or have timed out. Note this does not starts a movement,
    // it only reverses the waveform tables logically.
    while ( (cnt_pending > 0) && ((grid_state.interface_state == DS_CONNECTED)))
    {
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        cnt_pending = (grid_state.count_pending
                       + grid_state.num_queued);
    }


    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : reverseMotion():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }

    if (grid_state.count_timeout != old_count_timeout)
    {
        logGridState(config.logLevel, grid_state);

        LOG_CONTROL(LOG_ERROR, "%18.6f : reverseMotion():  error DE_CAN_COMMAND_TIMEOUT_ERROR, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    if (old_count_can_overflow != grid_state.count_can_overflow)
    {
        logGridState(config.logLevel, grid_state);

        LOG_CONTROL(LOG_ERROR, "%18.6f : reverseMotion():  error: firmware CAN buffer overflow\n",
                    ethercanif::get_realtime());
        return DE_FIRMWARE_CAN_BUFFER_OVERFLOW;
    }



    logGridState(config.logLevel, grid_state);

    LOG_CONTROL(LOG_INFO, "%18.6f : reverseMotion: command successfully sent OK\n",
                ethercanif::get_realtime());

    return DE_OK;

}




E_EtherCANErrCode AsyncInterface::abortMotionAsync(pthread_mutex_t & command_mutex,
        t_grid_state& grid_state,
        E_GridState& state_summary,
        t_fpuset const &fpuset)
{

    // NOTE: This command runs its first part without keeping
    // the command mutex, this enables it to interrupt
    // and preempt ongoing movements.

    // first, get current state of the grid
    state_summary = gateway.getGridState(grid_state);
    const unsigned long old_count_timeout = grid_state.count_timeout;
    const unsigned long old_count_can_overflow = grid_state.count_can_overflow;

    // check interface is connected
    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : abortMotion():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
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
                unique_ptr<CAN_Command> cmd(can_command.release());
                gateway.sendCommand(i, cmd);
            }
        }
    }

    // lock command mutex during waiting time for completion.
    // This prevents other commands from starting.
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
            && ((grid_state.interface_state == DS_CONNECTED)))
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

    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : abortMotion():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }


    if (grid_state.count_timeout != old_count_timeout)
    {
        logGridState(config.logLevel, grid_state);

        LOG_CONTROL(LOG_ERROR, "%18.6f : abortMotion():  error DE_CAN_COMMAND_TIMEOUT_ERROR\n",
                    ethercanif::get_realtime());
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    if (old_count_can_overflow != grid_state.count_can_overflow)
    {
        logGridState(config.logLevel, grid_state);

        LOG_CONTROL(LOG_ERROR, "%18.6f : abortMotion():  error: firmware CAN buffer overflow\n",
                    ethercanif::get_realtime());
        return DE_FIRMWARE_CAN_BUFFER_OVERFLOW;
    }



    logGridState(config.logLevel, grid_state);

    LOG_CONTROL(LOG_INFO, "%18.6f : abortMotion(): command successfully sent\n",
                ethercanif::get_realtime());
    return DE_OK;

}


E_EtherCANErrCode AsyncInterface::lockFPUAsync(int fpu_id, t_grid_state& grid_state,
        E_GridState& state_summary)
{


    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);

    const unsigned long old_count_timeout = grid_state.count_timeout;
    const unsigned long old_count_can_overflow = grid_state.count_can_overflow;

    // check interface is connected
    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : lockFPU():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }

    if ((fpu_id >= config.num_fpus) || (fpu_id < 0))
    {
        // the FPU id is out of range
        LOG_CONTROL(LOG_ERROR, "%18.6f : lockFPU():  error DE_INVALID_FPU_ID, FPU id out of range\n",
                    ethercanif::get_realtime());
        return DE_INVALID_FPU_ID;
    }


    t_fpu_state& fpu_state = grid_state.FPU_state[fpu_id];
    // we exclude moving FPUs and FPUs which are
    // searching datum.
    if ( (fpu_state.state == FPST_MOVING)
            && (fpu_state.state == FPST_DATUM_SEARCH))
    {
        // We do not allow lockng of moving FPUs.  (In
        // that case, the user should send an abortMotion command
        // first.)

        logGridState(config.logLevel, grid_state);

        LOG_CONTROL(LOG_ERROR, "%18.6f : lockFPU():  error DE_STILL_BUSY, "
                    "FPU # %i are still moving, if needed send abortMotion() first\n",
                    ethercanif::get_realtime(), fpu_id);
        return DE_STILL_BUSY;
    }


    unique_ptr<LockUnitCommand> can_command;
    can_command = gateway.provideInstance<LockUnitCommand>();
    const bool broadcast = false;
    can_command->parametrize(fpu_id, broadcast);
    unique_ptr<CAN_Command> cmd(can_command.release());
    gateway.sendCommand(fpu_id, cmd);


    int cnt_pending = 1;

    while ( (cnt_pending > 0) && ((grid_state.interface_state == DS_CONNECTED)))
    {
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        cnt_pending = (grid_state.count_pending
                       + grid_state.num_queued);
    }

    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : lockFPU():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }

    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : lockFPU():  error DE_CAN_COMMAND_TIMEOUT_ERROR\n",
                    ethercanif::get_realtime());
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    if (old_count_can_overflow != grid_state.count_can_overflow)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : lockFPU():  error: firmware CAN buffer overflow\n",
                    ethercanif::get_realtime());
        return DE_FIRMWARE_CAN_BUFFER_OVERFLOW;
    }


    logGridState(config.logLevel, grid_state);

    LOG_CONTROL(LOG_INFO, "%18.6f : lockFPU(): command successfully sent to FPU %i, is_locked = %i\n",
                ethercanif::get_realtime(), fpu_id, grid_state.FPU_state[fpu_id].is_locked);

    return DE_OK;
}


E_EtherCANErrCode AsyncInterface::unlockFPUAsync(int fpu_id, t_grid_state& grid_state,
        E_GridState& state_summary)
{



    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);

    const unsigned long old_count_timeout = grid_state.count_timeout;
    const unsigned long old_count_can_overflow = grid_state.count_can_overflow;

    // check interface is connected
    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : unlockFPU():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }

    if ((fpu_id >= config.num_fpus) || (fpu_id < 0))
    {
        // the FPU id is out of range
        LOG_CONTROL(LOG_ERROR, "%18.6f : unlockFPU():  error DE_INVALID_FPU_ID, FPU id out of range\n",
                    ethercanif::get_realtime());
        return DE_INVALID_FPU_ID;
    }


    // check state for being locked
    t_fpu_state& fpu_state = grid_state.FPU_state[fpu_id];

    if (fpu_state.state != FPST_LOCKED)
    {
        logGridState(config.logLevel, grid_state);

        LOG_CONTROL(LOG_ERROR, "%18.6f : unlockFPU():  error DE_INVALID_FPU_STATE, "
                    "can't unlock FPU %i, it is not locked\n",
                    ethercanif::get_realtime(), fpu_id);
        return DE_INVALID_FPU_STATE;
    }


    unique_ptr<UnlockUnitCommand> can_command;
    const bool broadcast = false;
    can_command = gateway.provideInstance<UnlockUnitCommand>();
    can_command->parametrize(fpu_id, broadcast);
    unique_ptr<CAN_Command> cmd(can_command.release());
    gateway.sendCommand(fpu_id, cmd);


    int cnt_pending = 1;

    while ( (cnt_pending > 0) && ((grid_state.interface_state == DS_CONNECTED)))
    {
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        cnt_pending = (grid_state.count_pending
                       + grid_state.num_queued);
    }

    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : unlockFPU():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }

    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : unlockFPU():  error DE_CAN_COMMAND_TIMEOUT_ERROR\n",
                    ethercanif::get_realtime());
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    if (old_count_can_overflow != grid_state.count_can_overflow)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : unlockFPU():  error: firmware CAN buffer overflow\n",
                    ethercanif::get_realtime());
        return DE_FIRMWARE_CAN_BUFFER_OVERFLOW;
    }


    logGridState(config.logLevel, grid_state);

    LOG_CONTROL(LOG_INFO, "%18.6f : unlockFPU(): command successfully sent to FPU %i, is_locked = %i\n",
                ethercanif::get_realtime(), fpu_id, grid_state.FPU_state[fpu_id].is_locked);

    return DE_OK;
}


E_EtherCANErrCode AsyncInterface::pingFPUsAsync(t_grid_state& grid_state,
        E_GridState& state_summary, t_fpuset const &fpuset)
{

    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);
    const unsigned long old_count_timeout = grid_state.count_timeout;
    const unsigned long old_count_can_overflow = grid_state.count_can_overflow;

    // check interface is connected
    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : pingFPUs():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }


    // All fpus which are not moving are pinged.

    // (we avoid bothering moving FPUs, they are resource-constrained
    // and this could trigger malfunction)

    int cnt_pending = 0;
    unique_ptr<PingFPUCommand> can_command;
    for (int i=0; i < config.num_fpus; i++)
    {
        t_fpu_state& fpu_state = grid_state.FPU_state[i];
        // we exclude moving FPUs, but include FPUs which are
        // searching datum.
        if (( ! ((fpu_state.state == FPST_DATUM_SEARCH)
                 || (fpu_state.state == FPST_MOVING))) && fpuset[i])
        {

            // We use a non-broadcast command instance. The advantage
            // of this is that he CAN protocol is able to reliably
            // detect whether this command was received - for a
            // broadcast command, this is not absolutely sure,
            // only one CAN receiver sends a confirmation.
            bool broadcast = false;
            can_command = gateway.provideInstance<PingFPUCommand>();

            can_command->parametrize(i, broadcast);
            unique_ptr<CAN_Command> cmd(can_command.release());
            gateway.sendCommand(i, cmd);
            cnt_pending++;

        }
    }

    // wait until all generated ping commands have been responded to
    // or have timed out.
    while ( (cnt_pending > 0) && ((grid_state.interface_state == DS_CONNECTED)))
    {
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        if (grid_state.interface_state != DS_CONNECTED)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : pingFPUs():  error DE_NO_CONNECTION, connection was lost\n",
                        ethercanif::get_realtime());
            return DE_NO_CONNECTION;
        }

        cnt_pending = (grid_state.count_pending
                       + grid_state.num_queued);
    }

    if (grid_state.count_timeout != old_count_timeout)
    {
        logGridState(config.logLevel, grid_state);

        LOG_CONTROL(LOG_ERROR, "%18.6f : pingFPUs():  error DE_CAN_COMMAND_TIMEOUT_ERROR\n",
                    ethercanif::get_realtime());
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    if (old_count_can_overflow != grid_state.count_can_overflow)
    {
        logGridState(config.logLevel, grid_state);

        LOG_CONTROL(LOG_ERROR, "%18.6f : pingFPUs():  error: firmware CAN buffer overflow\n",
                    ethercanif::get_realtime());
        return DE_FIRMWARE_CAN_BUFFER_OVERFLOW;
    }



    LOG_CONTROL(LOG_INFO, "%18.6f : pingFPUs(): command successfully completed\n",
                ethercanif::get_realtime());

    logGridState(config.logLevel, grid_state);
    return DE_OK;

}


E_EtherCANErrCode AsyncInterface::enableBetaCollisionProtectionAsync(t_grid_state& grid_state,
        E_GridState& state_summary)
{
    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);

    logGridState(config.logLevel, grid_state);

    const unsigned long old_count_timeout = grid_state.count_timeout;
    const unsigned long old_count_can_overflow = grid_state.count_can_overflow;

    // check interface is connected
    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : enableBetaCollisionProtection():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }


    {
        // make sure no FPU is moving or finding datum
        bool recoveryok=true;
        int moving_fpuid = -1;
        for (int i=0; i < config.num_fpus; i++)
        {
            t_fpu_state& fpu_state = grid_state.FPU_state[i];
            // we exclude moving FPUs and FPUs which are
            // searching datum.
            if ( (fpu_state.state == FPST_MOVING)
                    && (fpu_state.state == FPST_DATUM_SEARCH))
            {
                recoveryok = false;
                moving_fpuid = i;
                break;
            }
        }

        if (! recoveryok)
        {
            // We do not allow recovery when there are moving FPUs.  (In
            // that case, the user should send an abortMotion command
            // first.)
            LOG_CONTROL(LOG_ERROR, "%18.6f : enableBetaCollisionProtection():  error DE_STILL_BUSY, "
                        "FPU %i is still moving\n",
                        ethercanif::get_realtime(), moving_fpuid);
            return DE_STILL_BUSY;
        }
    }


    unique_ptr<EnableBetaCollisionProtectionCommand> can_command;
    for (int i=0; i < config.num_fpus; i++)
    {
        bool broadcast = false;
        can_command = gateway.provideInstance<EnableBetaCollisionProtectionCommand>();
        can_command->parametrize(i, broadcast);
        unique_ptr<CAN_Command> cmd(can_command.release());
        gateway.sendCommand(i, cmd);
    }

    int cnt_pending = config.num_fpus;

    while ( (cnt_pending > 0) && ((grid_state.interface_state == DS_CONNECTED)))
    {
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        cnt_pending = (grid_state.count_pending
                       + grid_state.num_queued);
    }

    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : enableBetaCollisionProtection():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }

    if (grid_state.count_timeout != old_count_timeout)
    {
        logGridState(config.logLevel, grid_state);

        LOG_CONTROL(LOG_ERROR, "%18.6f : enableBetaCollisionProtection():  error DE_CAN_COMMAND_TIMEOUT_ERROR\n",
                    ethercanif::get_realtime());
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    if (old_count_can_overflow != grid_state.count_can_overflow)
    {
        logGridState(config.logLevel, grid_state);

        LOG_CONTROL(LOG_ERROR, "%18.6f : enableBetaCollisionProtection():  error: firmware CAN buffer overflow\n",
                    ethercanif::get_realtime());
        return DE_FIRMWARE_CAN_BUFFER_OVERFLOW;
    }



    logGridState(config.logLevel, grid_state);

    LOG_CONTROL(LOG_INFO, "%18.6f : enableBetaCollisionProtection(): command successfully sent to grid\n",
                ethercanif::get_realtime());
    return DE_OK;
}

E_EtherCANErrCode AsyncInterface::freeBetaCollisionAsync(int fpu_id,
        E_REQUEST_DIRECTION request_dir,
        t_grid_state& grid_state,
        E_GridState& state_summary)
{
    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);

    const unsigned long old_count_timeout = grid_state.count_timeout;
    const unsigned long old_count_can_overflow = grid_state.count_can_overflow;

    // check interface is connected
    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : freeBetaCollision():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }

    if ((fpu_id >= config.num_fpus) || (fpu_id < 0))
    {
        // the FPU id is out of range
        LOG_CONTROL(LOG_ERROR, "%18.6f : freeBetaCollision():  error DE_INVALID_FPU_ID, FPU id out of range\n",
                    ethercanif::get_realtime());
        return DE_INVALID_FPU_ID;
    }


    // make sure no FPU is moving or finding datum
    {
        bool recoveryok=true;
        int moving_fpuid = -1;
        for (int i=0; i < config.num_fpus; i++)
        {
            t_fpu_state& fpu_state = grid_state.FPU_state[i];
            // we exclude moving FPUs and FPUs which are
            // searching datum.
            if ( (fpu_state.state == FPST_MOVING)
                    && (fpu_state.state == FPST_DATUM_SEARCH))
            {
                recoveryok = false;
                moving_fpuid = i;
                break;
            }
        }

        if (! recoveryok)
        {
            // We do not allow recovery when there are moving FPUs.  (In
            // that case, the user should send an abortMotion command
            // first.)

            logGridState(config.logLevel, grid_state);

            LOG_CONTROL(LOG_ERROR, "%18.6f : freeBetaCollision():  error DE_STILL_BUSY, "
                        "FPU %i is still moving, if needed send abortMotion() first\n",
                        ethercanif::get_realtime(), moving_fpuid);
            return DE_STILL_BUSY;
        }
    }


    unique_ptr<FreeBetaCollisionCommand> can_command;
    can_command = gateway.provideInstance<FreeBetaCollisionCommand>();
    can_command->parametrize(fpu_id, request_dir);
    unique_ptr<CAN_Command> cmd(can_command.release());
    gateway.sendCommand(fpu_id, cmd);


    int cnt_pending = 1;

    while ( (cnt_pending > 0) && ((grid_state.interface_state == DS_CONNECTED)))
    {
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        cnt_pending = (grid_state.count_pending
                       + grid_state.num_queued);
    }

    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : freeBetaCollision():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }

    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : freeBetaCollision():  error DE_CAN_COMMAND_TIMEOUT_ERROR\n",
                    ethercanif::get_realtime());
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    if (old_count_can_overflow != grid_state.count_can_overflow)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : freeBetaCollision():  error: firmware CAN buffer overflow\n",
                    ethercanif::get_realtime());
        return DE_FIRMWARE_CAN_BUFFER_OVERFLOW;
    }



    logGridState(config.logLevel, grid_state);

    LOG_CONTROL(LOG_INFO, "%18.6f : freeBetaCollision(): command successfully sent to FPU %i\n",
                ethercanif::get_realtime(), fpu_id);

    return DE_OK;
}


E_GridState AsyncInterface::getGridState(t_grid_state& out_state) const
{
    E_GridState state_summary = gateway.getGridState(out_state);
    logGridState(config.logLevel, out_state);
    return state_summary;
}

E_GridState AsyncInterface::waitForState(E_WaitTarget target,
        t_grid_state& out_detailed_state, double &max_wait_time, bool &cancelled) const
{
    return gateway.waitForState(target, out_detailed_state, max_wait_time, cancelled);
}



E_EtherCANErrCode AsyncInterface::setUStepLevelAsync(int ustep_level,
        t_grid_state& grid_state,
        E_GridState& state_summary,
        t_fpuset const &fpuset)
{
    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);

    const unsigned long old_count_timeout = grid_state.count_timeout;
    const unsigned long old_count_can_overflow = grid_state.count_can_overflow;

    // check interface is connected
    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : setUStepLevel():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
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
                    ethercanif::get_realtime(), ustep_level);
        return DE_INVALID_PAR_VALUE;
    }


    for (int i=0; i < config.num_fpus; i++)
    {
        t_fpu_state& fpu_state = grid_state.FPU_state[i];
        // we exclude moving FPUs and FPUs which are
        // searching datum.
        if ( fpu_state.state != FPST_UNINITIALIZED)
        {
            // FPU state does not allow command
            LOG_CONTROL(LOG_ERROR, "%18.6f : setUStepLevel():  error DE_INVALID_FPU_STATE, all FPUs "
                        "need to be in state FPST_UNINITIALIZED\n",
                        ethercanif::get_realtime());
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
        unique_ptr<CAN_Command> cmd(can_command.release());
        gateway.sendCommand(i, cmd);
        cnt_pending++;
    }

    // wait until all generated commands have been responded to
    // or have timed out.
    while ( (cnt_pending > 0) && ((grid_state.interface_state == DS_CONNECTED)))
    {
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        if (grid_state.interface_state != DS_CONNECTED)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : setUStepLevel():  error DE_NO_CONNECTION, connection was lost\n",
                        ethercanif::get_realtime());
            return DE_NO_CONNECTION;
        }

        cnt_pending = (grid_state.count_pending
                       + grid_state.num_queued);
    }

    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : setUStepLevel():  error DE_CAN_COMMAND_TIMEOUT_ERROR\n",
                    ethercanif::get_realtime());

        logGridState(config.logLevel, grid_state);

        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    if (old_count_can_overflow != grid_state.count_can_overflow)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : setUStepLevel():  error: firmware CAN buffer overflow\n",
                    ethercanif::get_realtime());

        logGridState(config.logLevel, grid_state);

        return DE_FIRMWARE_CAN_BUFFER_OVERFLOW;
    }



    logGridState(config.logLevel, grid_state);

    LOG_CONTROL(LOG_INFO, "%18.6f : setUStepLevel(): command successfully sent, ustep_level set to %i\n",
                ethercanif::get_realtime(), ustep_level);

    return DE_OK;

}

const char * str_interface_state(const E_InterfaceState interface_state)
{
    switch (interface_state)
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

void AsyncInterface::logGridState(const E_LogLevel logLevel, t_grid_state& grid_state) const
{
    char log_buffer[1024];
    char * logp = log_buffer;

    if (logLevel < LOG_INFO)
    {
        return;
    }

    double cur_time = ethercanif::get_realtime();
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
        logp += sprintf(logp, "interface state: DS=%s,  count_timeout=%lu, count_pending=%i, nqueued=%i",
                        str_interface_state(grid_state.interface_state),
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
                                (unsigned) (fpu.num_active_timeouts),
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


E_EtherCANErrCode AsyncInterface::readRegisterAsync(uint16_t read_address,
        t_grid_state& grid_state,
        E_GridState& state_summary,
        t_fpuset const &fpuset)
{

    // first, get current state of the grid
    state_summary = gateway.getGridState(grid_state);
    const unsigned long old_count_timeout = grid_state.count_timeout;
    const unsigned long old_count_can_overflow = grid_state.count_can_overflow;

    // check interface is connected
    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : readRegister():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
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
            unique_ptr<CAN_Command> cmd(can_command.release());
            qstate = gateway.sendCommand(i, cmd);
            assert(qstate == CommandQueue::QS_OK);
            num_pending++;

        }
    }


    // fpus are now responding in parallel.
    //
    // As long as any fpus need to respond, wait for
    // them to finish.
    while ( (num_pending > 0) && ((grid_state.interface_state == DS_CONNECTED)))
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

    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : readRegister():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }

    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : readRegister(): error: DE_CAN_COMMAND_TIMEOUT_ERROR.\n",
                    ethercanif::get_realtime());
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    if (old_count_can_overflow != grid_state.count_can_overflow)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : readRegister(): error: firmware CAN buffer overflow.\n",
                    ethercanif::get_realtime());
        return DE_FIRMWARE_CAN_BUFFER_OVERFLOW;
    }




    // log result if in debug mode
    if (config.logLevel >= LOG_DEBUG)
    {
        double log_time = ethercanif::get_realtime();
        for(int i=0; i < config.num_fpus; i++)
        {
            LOG_CONTROL(LOG_INFO, "%18.6f : readregister: FPU # %4i, location 0X%04x = 0X%02x.\n",
                        log_time, i, read_address, grid_state.FPU_state[i].register_value);

        }
    }

    LOG_CONTROL(LOG_INFO, "%18.6f : readRegister(): values were retrieved successfully.\n",
                ethercanif::get_realtime());

    return DE_OK;

}

// get minimum firmware version value, using cache when valid
E_EtherCANErrCode AsyncInterface::assureMinFirmwareVersion(const int req_fw_major,
        const int req_fw_minor,
        const int req_fw_patch,
        const char* caller_name,
        t_fpuset const &fpuset,
        t_grid_state& grid_state)
{

    E_GridState state_summary;
    uint8_t min_firmware_version[3];
    int min_firmware_fpu;

    E_EtherCANErrCode ecode = getMinFirmwareVersionAsync(fpuset, min_firmware_version, min_firmware_fpu, grid_state, state_summary);

    if (ecode != DE_OK)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : %s: error: retrieving firmware version failed with error code %i\n",
                    ethercanif::get_realtime(), caller_name, ecode);
        return ecode;
    }


    {
        if ( (min_firmware_version[0] < req_fw_major)
                || ( (min_firmware_version[0] == req_fw_major) && (min_firmware_version[1] < req_fw_minor))
                || ( (min_firmware_version[0] == req_fw_major) && (min_firmware_version[1] == req_fw_minor)
                     && (min_firmware_version[2] < req_fw_patch)))
        {
            // the firmware does not implement what we need
            LOG_CONTROL(LOG_ERROR, "%18.6f : %s: error: DE_FIRMWARE_UNIMPLEMENTED"
                        " command requires firmware version >= %i.%i.%i,"
                        " version %i.%i.%i found in FPU %i\n",
                        ethercanif::get_realtime(), caller_name,
                        req_fw_major, req_fw_minor, req_fw_patch,
                        min_firmware_version[0], min_firmware_version[1], min_firmware_version[2], min_firmware_fpu);
            return DE_FIRMWARE_UNIMPLEMENTED;
        }

    }

    return DE_OK;
}

// get minimum firmware version value, using cache when valid, otherwise query FPUs
E_EtherCANErrCode AsyncInterface::getMinFirmwareVersionAsync(t_fpuset const &fpuset,
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
        E_EtherCANErrCode ecode = getFirmwareVersionAsync(grid_state, state_summary, fpuset);
        if (ecode != DE_OK)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: findDatum(): "
                        "could not retrieve firmware versions - command cancelled\n",
                        ethercanif::get_realtime());
            return ecode;
        }

        getCachedMinFirmwareVersion(fpuset,
                                    successfully_retrieved,
                                    min_firmware_version,
                                    min_firmware_fpu);

        if (! successfully_retrieved)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : AsyncInterface: findDatum(): "
                        "could not retrieve firmware versions - command cancelled\n",
                        ethercanif::get_realtime());
            return ecode;
        }
    }
    return DE_OK;
}

void AsyncInterface::getCachedMinFirmwareVersion(t_fpuset const &fpuset,
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
        for (int k=0; k < 3; k++)
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

E_EtherCANErrCode AsyncInterface::getFirmwareVersionAsync(t_grid_state& grid_state,
        E_GridState& state_summary,
        t_fpuset const &fpuset)
{



    // first, get current state of the grid
    state_summary = gateway.getGridState(grid_state);
    const unsigned long old_count_timeout = grid_state.count_timeout;
    const unsigned long old_count_can_overflow = grid_state.count_can_overflow;

    // check interface is connected
    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : getFirmwareVersion():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }



    unique_ptr<GetFirmwareVersionCommand> can_command;
    int num_pending = 0;
    for (int i=0; i < config.num_fpus; i++)
    {
        // we exclude locked FPUs
        if ((! gateway.isLocked(i) ) && fpuset[i])
        {
            can_command = gateway.provideInstance<GetFirmwareVersionCommand>();
            assert(can_command);
            bool broadcast = false;
            can_command->parametrize(i, broadcast);
            // send the command (the actual sending happens
            // in the TX thread in the background).
            CommandQueue::E_QueueState qstate;
            unique_ptr<CAN_Command> cmd(can_command.release());
            qstate = gateway.sendCommand(i, cmd);
            assert(qstate == CommandQueue::QS_OK);
            num_pending++;

        }
    }


    // fpus are now responding in parallel.
    //
    // As long as any fpus need to respond, wait for
    // them to finish.
    while ( (num_pending > 0) && ((grid_state.interface_state == DS_CONNECTED)))
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

    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : getFirmwareVersion():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }

    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : getFirmwareVersion(): error: DE_CAN_COMMAND_TIMEOUT_ERROR.\n",
                    ethercanif::get_realtime());
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    if (old_count_can_overflow != grid_state.count_can_overflow)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : getFirmwareVersion(): error: firmware CAN buffer overflow.\n",
                    ethercanif::get_realtime());
        return DE_FIRMWARE_CAN_BUFFER_OVERFLOW;
    }



    // log result if in debug mode
    if (config.logLevel >= LOG_INFO)
    {
        double log_time = ethercanif::get_realtime();
        for(int i=0; i < config.num_fpus; i++)
        {
            LOG_CONTROL(LOG_INFO, "%18.6f : getFirmwareVersion: FPU # %4i, retrieved firmware version = %i.%i.%i.\n",
                        log_time, i,
                        grid_state.FPU_state[i].firmware_version[0],
                        grid_state.FPU_state[i].firmware_version[1],
                        grid_state.FPU_state[i].firmware_version[2]);
        }
    }

    LOG_CONTROL(LOG_INFO, "%18.6f : getFirmwareVersion(): values were retrieved successfully.\n",
                ethercanif::get_realtime());




    // copy data from grid_state structure to internal cache.  This is
    // done because the firmware version usually needs to be known
    // before a command is executed, and we want to avoid duplicated
    // state retrievals.
    for (int i=0; i < config.num_fpus; i++)
    {
        if (! fpuset[i])
        {
            continue;
        }

        memcpy(fpu_firmware_version[i],
               grid_state.FPU_state[i].firmware_version,
               sizeof(grid_state.FPU_state[i].firmware_version));
    }

    LOG_CONTROL(LOG_INFO, "%18.6f : getFirmwareVersion(): retrieved firmware versions successfully\n",
                ethercanif::get_realtime());

    return DE_OK;

}

E_EtherCANErrCode AsyncInterface::readSerialNumbersAsync(t_grid_state& grid_state,
        E_GridState& state_summary, t_fpuset const &fpuset)
{

    // first, get current state of the grid
    state_summary = gateway.getGridState(grid_state);
    const unsigned long old_count_timeout = grid_state.count_timeout;
    const unsigned long old_count_can_overflow = grid_state.count_can_overflow;

    // check interface is connected
    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : readSerialNumbers():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }

    uint8_t min_firmware_version[3] = {0,0,0};
    int min_firmware_fpu = -1;

    E_EtherCANErrCode ecode = getMinFirmwareVersionAsync(fpuset, min_firmware_version, min_firmware_fpu, grid_state, state_summary);

    if (ecode != DE_OK)
    {
        return ecode;
    }

#if (CAN_PROTOCOL_VERSION == 1)
    if ((min_firmware_version[0] < 1)
            ||(min_firmware_version[1] < 3))
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : readSerialNumbers():  error DE_FIRMWARE_UNIMPLEMENTED "
                    "- FPU firmware does not provide feature\n",
                    ethercanif::get_realtime());
        return DE_FIRMWARE_UNIMPLEMENTED;
    }
#endif



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
        unique_ptr<CAN_Command> cmd(can_command.release());
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
    while ( (num_pending > 0) && ((grid_state.interface_state == DS_CONNECTED)))
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

    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : readSerialNumbers():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }


    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : readSerialNumbers():  error DE_CAN_COMMAND_TIMEOUT_ERROR\n",
                    ethercanif::get_realtime());

        logGridState(config.logLevel, grid_state);

        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    if (old_count_can_overflow != grid_state.count_can_overflow)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : readSerialNumbers():  error: firmware CAN buffer overflow\n",
                    ethercanif::get_realtime());

        logGridState(config.logLevel, grid_state);
        return DE_FIRMWARE_CAN_BUFFER_OVERFLOW;
    }



    logGridState(config.logLevel, grid_state);

    LOG_CONTROL(LOG_INFO, "%18.6f : readSerialNumbers(): retrieved serial numbers\n",
                ethercanif::get_realtime());
    for(int i=0; i < config.num_fpus; i++)
    {
        const double t = ethercanif::get_realtime();
        if (fpuset[i])
        {
            LOG_CONTROL(LOG_INFO, "%18.6f : FPU %i : SN = %s\n",
                        t, i, grid_state.FPU_state[i].serial_number);
        }
    }
    return DE_OK;

}


E_EtherCANErrCode AsyncInterface::writeSerialNumberAsync(int fpu_id, const char serial_number[],
        t_grid_state& grid_state,
        E_GridState& state_summary)
{
    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);

    const unsigned long old_count_timeout = grid_state.count_timeout;
    const unsigned long old_count_can_overflow = grid_state.count_can_overflow;

    // check interface is connected
    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : writeSerialNumber():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }

    if ((fpu_id >= config.num_fpus) || (fpu_id < 0))
    {
        // the FPU id is out of range
        LOG_CONTROL(LOG_ERROR, "%18.6f : writeSerialNumber():  error DE_INVALID_FPU_ID, FPU id out of range\n",
                    ethercanif::get_realtime());
        return DE_INVALID_FPU_ID;
    }

    const int sn_len = strnlen(serial_number, LEN_SERIAL_NUMBER);
    if (sn_len == LEN_SERIAL_NUMBER)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : writeSerialNumber():  error DE_INVALID_PAR_VALUE,"
                    " serial number is too long (length %i, only %i characters allowed)\n",
                    ethercanif::get_realtime(), sn_len, LEN_SERIAL_NUMBER-1);

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
                        ethercanif::get_realtime());
            return DE_INVALID_PAR_VALUE;
        }
    }

    t_fpuset fpuset;
    for(int i = 0; i < config.num_fpus; i++)
    {
        fpuset[i]=true;
    }
    // get movement state
    E_EtherCANErrCode ecode = pingFPUsAsync(grid_state, state_summary, fpuset);

    if (ecode != DE_OK)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : pingFPUs failed, aborting writeSerialNumber() command \n",
                    ethercanif::get_realtime());
        return ecode;
    }

    uint8_t min_firmware_version[3] = {0,0,0};
    int min_firmware_fpu = -1;
    ecode = getMinFirmwareVersionAsync(fpuset, min_firmware_version, min_firmware_fpu, grid_state, state_summary);


    // get all existing numbers
    ecode = readSerialNumbersAsync(grid_state, state_summary, fpuset);

    if (ecode != DE_OK)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : readSerialNumbers failed, aborting writeSerialNumber() command \n",
                    ethercanif::get_realtime());
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
                    ethercanif::get_realtime());
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
                        ethercanif::get_realtime());
            return DE_DUPLICATE_SERIAL_NUMBER;

        }
    }


    unique_ptr<WriteSerialNumberCommand> can_command;
    can_command = gateway.provideInstance<WriteSerialNumberCommand>();
    can_command->parametrize(fpu_id, serial_number);
    unique_ptr<CAN_Command> cmd(can_command.release());
    gateway.sendCommand(fpu_id, cmd);


    int cnt_pending = 1;

    while ( (cnt_pending > 0) && ((grid_state.interface_state == DS_CONNECTED)))
    {
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        cnt_pending = (grid_state.count_pending + grid_state.num_queued);
    }

    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : writeSerialNumber():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }

    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : writeSerialNumber():  error DE_CAN_COMMAND_TIMEOUT_ERROR\n",
                    ethercanif::get_realtime());
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    if (old_count_can_overflow != grid_state.count_can_overflow)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : writeSerialNumber():  error: firmware CAN buffer overflow\n",
                    ethercanif::get_realtime());
        return DE_FIRMWARE_CAN_BUFFER_OVERFLOW;
    }



    logGridState(config.logLevel, grid_state);

    LOG_CONTROL(LOG_INFO, "%18.6f : writeSerialNumber(): FPU %i: serial number '%s' successfully written to FPU\n",
                ethercanif::get_realtime(), fpu_id, serial_number);

    return DE_OK;
}

E_EtherCANErrCode AsyncInterface::enableMoveAsync(int fpu_id,
        t_grid_state& grid_state,
        E_GridState& state_summary)
{
    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);

    const unsigned long old_count_timeout = grid_state.count_timeout;
    const unsigned long old_count_can_overflow = grid_state.count_can_overflow;

    // check interface is connected
    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : enableMove():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }

    if ((fpu_id >= config.num_fpus) || (fpu_id < 0))
    {
        // the FPU id is out of range
        LOG_CONTROL(LOG_ERROR, "%18.6f : enableMove():  error DE_INVALID_FPU_ID, FPU id out of range\n",
                    ethercanif::get_realtime());
        return DE_INVALID_FPU_ID;
    }


    // make sure no FPU is moving or finding datum
    bool enableok=true;
    int fpuid_moving = -1;
    for (int i=0; i < config.num_fpus; i++)
    {
        t_fpu_state& fpu_state = grid_state.FPU_state[i];
        // we exclude moving FPUs and FPUs which are
        // searching datum.
        if ( (fpu_state.state == FPST_MOVING)
                && (fpu_state.state == FPST_DATUM_SEARCH))
        {
            enableok = false;
            fpuid_moving = i;
            break;
        }
    }

    if (! enableok)
    {
        // We do not allow recovery when there are any moving FPUs.  (In
        // that case, the user should send an abortMotion command
        // first.)

        logGridState(config.logLevel, grid_state);

        LOG_CONTROL(LOG_ERROR, "%18.6f : enableMove():  error DE_STILL_BUSY, "
                    "FPU %i is still moving, if needed send abortMotion() first\n",
                    ethercanif::get_realtime(), fpuid_moving);
        return DE_STILL_BUSY;
    }


    unique_ptr<EnableMoveCommand> can_command;
    can_command = gateway.provideInstance<EnableMoveCommand>();
    const bool broadcast = false;
    can_command->parametrize(fpu_id, broadcast);
    unique_ptr<CAN_Command> cmd(can_command.release());
    gateway.sendCommand(fpu_id, cmd);


    int cnt_pending = 1;

    while ( (cnt_pending > 0) && ((grid_state.interface_state == DS_CONNECTED)))
    {
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        cnt_pending = (grid_state.count_pending
                       + grid_state.num_queued);
    }

    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : enableMove():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }

    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : enableMove():  error DE_CAN_COMMAND_TIMEOUT_ERROR\n",
                    ethercanif::get_realtime());
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    if (old_count_can_overflow != grid_state.count_can_overflow)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : enableMove():  error: firmware CAN buffer overflow\n",
                    ethercanif::get_realtime());
        return DE_FIRMWARE_CAN_BUFFER_OVERFLOW;
    }



    logGridState(config.logLevel, grid_state);

    LOG_CONTROL(LOG_INFO, "%18.6f : enableMove(): command successfully sent to FPU %i\n",
                ethercanif::get_realtime(), fpu_id);

    return DE_OK;
}




E_EtherCANErrCode AsyncInterface::resetStepCounterAsync(t_grid_state& grid_state,
        E_GridState& state_summary, t_fpuset const &fpuset)
{
    LOG_CONTROL(LOG_INFO, "%18.6f : resetting FPU stepcounters\n",
                ethercanif::get_realtime());

    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);
    const unsigned long old_count_timeout = grid_state.count_timeout;
    const unsigned long old_count_can_overflow = grid_state.count_can_overflow;
    // check interface is connected
    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : resetStepCounter() error: interface is not connected, can't reset FPUs\n",
                    ethercanif::get_realtime());
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
        // we exclude moving FPUs abd FPUs which are
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
                    ethercanif::get_realtime());
        return DE_STILL_BUSY;
    }


    int cnt_pending=0;
    unique_ptr<ResetStepCounterCommand> can_command;
    for (int i=0; i < config.num_fpus; i++)
    {
        if (!fpuset[i])
        {
            continue;
        }
        bool broadcast = false;
        can_command = gateway.provideInstance<ResetStepCounterCommand>();
        can_command->parametrize(i, broadcast);
        unique_ptr<CAN_Command> cmd(can_command.release());
        gateway.sendCommand(i, cmd);
        cnt_pending++;
    }


    while ( (cnt_pending > 0) && ((grid_state.interface_state == DS_CONNECTED)))
    {
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        cnt_pending = (grid_state.count_pending + grid_state.num_queued);
    }


    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : error: interface is not connected, can't reset step counters\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }

    // It is important to compare for inequality here, because
    // count_timeout is an unsigned value which can intentionally wrap
    // without causing undefined behaviour.
    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : resetStepCounter():  error DE_CAN_COMMAND_TIMEOUT_ERROR\n",
                    ethercanif::get_realtime());
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    if (old_count_can_overflow != grid_state.count_can_overflow)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : resetStepCounter():  error: firmware CAN buffer overflow\n",
                    ethercanif::get_realtime());

        return DE_FIRMWARE_CAN_BUFFER_OVERFLOW;
    }

    LOG_CONTROL(LOG_INFO, "%18.6f : resetStepCounter: command completed succesfully\n",
                ethercanif::get_realtime());

    logGridState(config.logLevel, grid_state);

    return DE_OK;

}



E_EtherCANErrCode AsyncInterface::enableAlphaLimitProtectionAsync(t_grid_state& grid_state,
        E_GridState& state_summary)
{
    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);

    logGridState(config.logLevel, grid_state);

    const unsigned long old_count_timeout = grid_state.count_timeout;
    const unsigned long old_count_can_overflow = grid_state.count_can_overflow;

    // check interface is connected
    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : enableAlphaLimitProtection():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
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
        LOG_CONTROL(LOG_ERROR, "%18.6f : enableAlphaLimitProtection():  error DE_STILL_BUSY, "
                    "FPUs are still moving\n",
                    ethercanif::get_realtime());
        return DE_STILL_BUSY;
    }


    unique_ptr<EnableAlphaLimitProtectionCommand> can_command;
    for (int i=0; i < config.num_fpus; i++)
    {
        bool broadcast = false;
        can_command = gateway.provideInstance<EnableAlphaLimitProtectionCommand>();
        can_command->parametrize(i, broadcast);
        unique_ptr<CAN_Command> cmd(can_command.release());
        gateway.sendCommand(i, cmd);
    }

    int cnt_pending = config.num_fpus;

    while ( (cnt_pending > 0) && ((grid_state.interface_state == DS_CONNECTED)))
    {
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        cnt_pending = (grid_state.count_pending
                       + grid_state.num_queued);
    }

    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : enableAlphaLimitProtection():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }

    if (grid_state.count_timeout != old_count_timeout)
    {
        logGridState(config.logLevel, grid_state);

        LOG_CONTROL(LOG_ERROR, "%18.6f : enableAlphaLimitProtection():  error DE_CAN_COMMAND_TIMEOUT_ERROR\n",
                    ethercanif::get_realtime());
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    if (old_count_can_overflow != grid_state.count_can_overflow)
    {
        logGridState(config.logLevel, grid_state);

        LOG_CONTROL(LOG_ERROR, "%18.6f : enableAlphaLimitProtection():  error: firmware CAN buffer overflow\n",
                    ethercanif::get_realtime());
        return DE_FIRMWARE_CAN_BUFFER_OVERFLOW;
    }



    logGridState(config.logLevel, grid_state);

    LOG_CONTROL(LOG_INFO, "%18.6f : enableAlphaLimitProtection(): command successfully sent to grid\n",
                ethercanif::get_realtime());
    return DE_OK;
}

E_EtherCANErrCode AsyncInterface::freeAlphaLimitBreachAsync(int fpu_id, E_REQUEST_DIRECTION request_dir,
        t_grid_state& grid_state,
        E_GridState& state_summary)
{
    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);

    const unsigned long old_count_timeout = grid_state.count_timeout;
    const unsigned long old_count_can_overflow = grid_state.count_can_overflow;

    // check interface is connected
    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : freeAlphaLimitBreach():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }

    if ((fpu_id >= config.num_fpus) || (fpu_id < 0))
    {
        // the FPU id is out of range
        LOG_CONTROL(LOG_ERROR, "%18.6f : freeAlphaLimitBreach():  error DE_INVALID_FPU_ID, FPU id out of range\n",
                    ethercanif::get_realtime());
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

        LOG_CONTROL(LOG_ERROR, "%18.6f : freeAlphaLimitBreach():  error DE_STILL_BUSY, "
                    "FPUs are still moving, if needed send abortMotion() first()\n",
                    ethercanif::get_realtime());
        return DE_STILL_BUSY;
    }


    unique_ptr<FreeAlphaLimitBreachCommand> can_command;
    can_command = gateway.provideInstance<FreeAlphaLimitBreachCommand>();
    can_command->parametrize(fpu_id, request_dir);
    unique_ptr<CAN_Command> cmd(can_command.release());
    gateway.sendCommand(fpu_id, cmd);


    int cnt_pending = 1;

    while ( (cnt_pending > 0) && ((grid_state.interface_state == DS_CONNECTED)))
    {
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        cnt_pending = (grid_state.count_pending
                       + grid_state.num_queued);
    }

    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : freeAlphaLimitBreach():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }

    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : freeAlphaLimitBreach():  error DE_CAN_COMMAND_TIMEOUT_ERROR\n",
                    ethercanif::get_realtime());
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    if (old_count_can_overflow != grid_state.count_can_overflow)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : freeAlphaLimitBreach():  error: firmware CAN buffer overflow\n",
                    ethercanif::get_realtime());
        return DE_FIRMWARE_CAN_BUFFER_OVERFLOW;
    }



    logGridState(config.logLevel, grid_state);

    LOG_CONTROL(LOG_INFO, "%18.6f : freeAlphaLimitBreach(): command successfully sent to FPU %i\n",
                ethercanif::get_realtime(), fpu_id);

    return DE_OK;
}


E_EtherCANErrCode AsyncInterface::setStepsPerSegmentAsync(int minsteps,
        int maxsteps,
        t_grid_state& grid_state,
        E_GridState& state_summary,
        t_fpuset const &fpuset)
{
    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);

    const unsigned long old_count_timeout = grid_state.count_timeout;
    const unsigned long old_count_can_overflow = grid_state.count_can_overflow;

    // check interface is connected
    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : setStepsPerSegment():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }

    if ( (minsteps <= 0) || (minsteps > maxsteps) || (maxsteps > 5000))
    {
        // value is not allowed
        LOG_CONTROL(LOG_ERROR, "%18.6f : setStepsPerSegment():  error DE_INVALID_PAR_VALUE, value %i .. %i not allowed\n",
                    ethercanif::get_realtime(), minsteps, maxsteps);
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
            LOG_CONTROL(LOG_ERROR, "%18.6f : setStepsPerSegment():  error DE_INVALID_FPU_STATE, all FPUs "
                        "need to be in state FPST_UNINITIALIZED\n",
                        ethercanif::get_realtime());
            return DE_INVALID_FPU_STATE;
        }
    }


    int cnt_pending = 0;
    unique_ptr<SetStepsPerSegmentCommand> can_command;
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
        can_command = gateway.provideInstance<SetStepsPerSegmentCommand>();

        can_command->parametrize(i, minsteps, maxsteps, broadcast);
        unique_ptr<CAN_Command> cmd(can_command.release());
        gateway.sendCommand(i, cmd);
        cnt_pending++;
    }

    // wait until all generated commands have been responded to
    // or have timed out.
    while ( (cnt_pending > 0) && ((grid_state.interface_state == DS_CONNECTED)))
    {
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        if (grid_state.interface_state != DS_CONNECTED)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : setStepsPerSegment():  error DE_NO_CONNECTION, connection was lost\n",
                        ethercanif::get_realtime());
            return DE_NO_CONNECTION;
        }

        cnt_pending = (grid_state.count_pending
                       + grid_state.num_queued);
    }

    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : setStepsPerSegment():  error DE_CAN_COMMAND_TIMEOUT_ERROR\n",
                    ethercanif::get_realtime());

        logGridState(config.logLevel, grid_state);

        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    if (old_count_can_overflow != grid_state.count_can_overflow)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : setStepsPerSegment():  error: firmware CAN buffer overflow\n",
                    ethercanif::get_realtime());

        logGridState(config.logLevel, grid_state);

        return DE_FIRMWARE_CAN_BUFFER_OVERFLOW;
    }



    logGridState(config.logLevel, grid_state);

    LOG_CONTROL(LOG_INFO, "%18.6f : setStepsPerSegment(): command successfully sent, steps per segment set to %i .. %i\n",
                ethercanif::get_realtime(), minsteps, maxsteps);

    return DE_OK;

}

E_EtherCANErrCode AsyncInterface::setTicksPerSegmentAsync(unsigned
        long ticks,
        t_grid_state& grid_state,
        E_GridState& state_summary,
        t_fpuset const &fpuset)
{
    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);

    const unsigned long old_count_timeout = grid_state.count_timeout;
    const unsigned long old_count_can_overflow = grid_state.count_can_overflow;

    // check interface is connected
    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : setTicksPerSegment():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }

    if ( (ticks <= 1000) || (ticks > 1e10))
    {
        // value is not allowed
        LOG_CONTROL(LOG_ERROR, "%18.6f : setTicksPerSegment():  error DE_INVALID_PAR_VALUE, value %lu  not allowed\n",
                    ethercanif::get_realtime(), ticks);
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
            LOG_CONTROL(LOG_ERROR, "%18.6f : setTicksPerSegment():  error DE_INVALID_FPU_STATE, all FPUs "
                        "need to be in state FPST_UNINITIALIZED\n",
                        ethercanif::get_realtime());
            return DE_INVALID_FPU_STATE;
        }
    }


    int cnt_pending = 0;
    unique_ptr<SetTicksPerSegmentCommand> can_command;
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
        can_command = gateway.provideInstance<SetTicksPerSegmentCommand>();

        can_command->parametrize(i, ticks, broadcast);
        unique_ptr<CAN_Command> cmd(can_command.release());
        gateway.sendCommand(i, cmd);
        cnt_pending++;
    }

    // wait until all generated commands have been responded to
    // or have timed out.
    while ( (cnt_pending > 0) && ((grid_state.interface_state == DS_CONNECTED)))
    {
        double max_wait_time = -1;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        if (grid_state.interface_state != DS_CONNECTED)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : setTicksPerSegment():  error DE_NO_CONNECTION, connection was lost\n",
                        ethercanif::get_realtime());
            return DE_NO_CONNECTION;
        }

        cnt_pending = (grid_state.count_pending
                       + grid_state.num_queued);
    }

    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : setTicksPerSegment():  error DE_CAN_COMMAND_TIMEOUT_ERROR\n",
                    ethercanif::get_realtime());

        logGridState(config.logLevel, grid_state);

        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    if (old_count_can_overflow != grid_state.count_can_overflow)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : setTicksPerSegment():  error: firmware CAN buffer overflow\n",
                    ethercanif::get_realtime());

        logGridState(config.logLevel, grid_state);

        return DE_FIRMWARE_CAN_BUFFER_OVERFLOW;
    }



    logGridState(config.logLevel, grid_state);

    LOG_CONTROL(LOG_INFO, "%18.6f : setTicksPerSegment(): command successfully sent, ticks per segment set to %lu",
                ethercanif::get_realtime(), ticks);

    return DE_OK;

}



E_EtherCANErrCode AsyncInterface::checkIntegrityAsync(t_grid_state& grid_state,
        E_GridState& state_summary,
        t_fpuset const &fpuset)
{
    // first, get current state of the grid
    state_summary = gateway.getGridState(grid_state);
    const unsigned long old_count_timeout = grid_state.count_timeout;
    const unsigned long old_count_can_overflow = grid_state.count_can_overflow;

    // check that interface is connected
    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : checkIntegrity():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }



    unique_ptr<CheckIntegrityCommand> can_command;
    int num_pending = 0;
    for (int i=0; i < config.num_fpus; i++)
    {
        // we exclude locked FPUs
        if ((! gateway.isLocked(i) ) && fpuset[i])
        {
            can_command = gateway.provideInstance<CheckIntegrityCommand>();
            assert(can_command);
            bool broadcast = false;
            can_command->parametrize(i, broadcast);
            // send the command (the actual sending happens
            // in the TX thread in the background).
            CommandQueue::E_QueueState qstate;
            unique_ptr<CAN_Command> cmd(can_command.release());
            qstate = gateway.sendCommand(i, cmd);
            assert(qstate == CommandQueue::QS_OK);
            num_pending++;

        }
    }


    // fpus are now responding in parallel. This command might take a while.
    //
    // As long as any fpus need to respond, wait for
    // them to finish.
    while ( (num_pending > 0) && ((grid_state.interface_state == DS_CONNECTED)))
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

    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : checkIntegrity():  error DE_NO_CONNECTION, connection was lost\n",
                    ethercanif::get_realtime());
        return DE_NO_CONNECTION;
    }

    if (grid_state.count_timeout != old_count_timeout)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : checkIntegrity(): error: DE_CAN_COMMAND_TIMEOUT_ERROR.\n",
                    ethercanif::get_realtime());
        return DE_CAN_COMMAND_TIMEOUT_ERROR;
    }

    if (old_count_can_overflow != grid_state.count_can_overflow)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : checkIntegrity(): error: firmware CAN buffer overflow.\n",
                    ethercanif::get_realtime());
        return DE_FIRMWARE_CAN_BUFFER_OVERFLOW;
    }



    // log result if in debug mode
    if (config.logLevel >= LOG_DEBUG)
    {
        double log_time = ethercanif::get_realtime();
        for(int i=0; i < config.num_fpus; i++)
        {
            LOG_CONTROL(LOG_INFO, "%18.6f : checkIntegrity: FPU # %4i : CRC32 checksum 0X%04x.\n",
                        log_time, i, grid_state.FPU_state[i].crc32);

        }
    }

    LOG_CONTROL(LOG_INFO, "%18.6f : checkIntegrity(): values were retrieved successfully.\n",
                ethercanif::get_realtime());

    return DE_OK;

}



}

} // end of namespace
