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
#include "canlayer/commands/RepeatMotionCommand.h"
#include "canlayer/commands/ResetFPUCommand.h"
#include "canlayer/commands/ReverseMotionCommand.h"
#include "canlayer/commands/SetUStepLevelCommand.h"


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



E_DriverErrCode AsyncDriver::initializeGridAsync(t_grid_state& grid_state,
        E_GridState& state_summary)
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

E_DriverErrCode AsyncDriver::resetFPUsAsync(t_grid_state& grid_state,
        E_GridState& state_summary)
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


    unique_ptr<ResetFPUCommand> can_command;
    for (int i=0; i < config.num_fpus; i++)
    {
        bool broadcast = false;
        can_command = gateway.provideInstance<ResetFPUCommand>();
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

E_DriverErrCode AsyncDriver::startAutoFindDatumAsync(t_grid_state& grid_state,
        E_GridState& state_summary,
        E_DATUM_SELECTION arm_selection)
{

    {
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
            break;
        }

        LOG_CONTROL(LOG_INFO, "%18.6f : AsyncDriver: findDatum started, arm_selection=%s\n",
                    canlayer::get_realtime(), as_string);
    }

    // first, get current state and time-out count of the grid
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
        break;
    default:
        LOG_CONTROL(LOG_ERROR, "%18.6f : findDatum(): error: invalid arm selection\n",
                    canlayer::get_realtime());
        return DE_INVALID_PAR_VALUE;
    }

    E_DriverErrCode ecode = pingFPUsAsync(grid_state, state_summary);

    if (ecode != DE_OK)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : ping failed, aborting findDatum()operation \n",
                    canlayer::get_realtime());
        return ecode;
    }

    // check no FPUs have ongoing collisions
    for (int i=0; i < config.num_fpus; i++)
    {
        E_FPU_STATE fpu_status = grid_state.FPU_state[i].state;
        if (fpu_status == FPST_OBSTACLE_ERROR)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : unresolved collisions - aborting findDatum()operation \n",
                        canlayer::get_realtime());
            return DE_UNRESOLVED_COLLISION;
        }
        if (fpu_status == FPST_ABORTED)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : FPUs are in aborted state - cancelling findDatum()operation \n",
                        canlayer::get_realtime());
            return DE_ABORTED_STATE;
        }
    }

    // check that beta arms are in allowed half-plane
    for (int i=0; i < config.num_fpus; i++)
    {
        const int BETA_DATUM_LIMIT = -5 * STEPS_PER_DEGREE_BETA;
        int beta_steps = grid_state.FPU_state[i].beta_steps;
        if (beta_steps < BETA_DATUM_LIMIT)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : beta arm appears to be in negative position - aborting findDatum()operation \n",
                        canlayer::get_realtime());
            return DE_UNIMPLEMENTED;
        }
    }

    // All fpus which are allowed to move, are moved automatically
    // until they hit the datum switch.

    unique_ptr<FindDatumCommand> can_command;
    for (int i=0; i < config.num_fpus; i++)
    {
        t_fpu_state& fpu_state = grid_state.FPU_state[i];
        if ( (fpu_state.state != FPST_UNINITIALIZED)
                || (fpu_state.state != FPST_AT_DATUM)
                || (fpu_state.state != FPST_READY_FORWARD)
                || (fpu_state.state != FPST_READY_BACKWARD)
                || (fpu_state.state != FPST_RESTING))

        {

            bool broadcast = false;
            can_command = gateway.provideInstance<FindDatumCommand>();
#if (CAN_PROTOCOL_VERSION == 1)
            can_command->parametrize(i, broadcast, arm_selection);
#else
            bool auto_datum = true;
            bool clockwise_first = false; // only relevant for non-auto
            can_command->parametrize(i, broadcast, auto_datum, clockwise_first, arm_selection);
#endif
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
        double &max_wait_time, bool &finished)
{
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
            LOG_CONTROL(LOG_ERROR, "%18.6f : waitFindDatum(): error: FPU movement was aborted for FPU %i\n",
                        canlayer::get_realtime(), i);
            logGridState(config.logLevel, grid_state);
            fsync(config.fd_controllog);

            return DE_ABORTED_STATE;
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
        const int MIN_STEPS, const int MAX_STEPS,
        const unsigned int MAX_NUM_SECTIONS, const double MAX_INCREASE) const
{

    LOG_CONTROL(LOG_INFO, "%18.6f : AsyncDriver: validating waveforms\n",
                canlayer::get_realtime());

    const int num_loading =  waveforms.size();
    const unsigned int num_steps = waveforms[0].steps.size();


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
                const double MAX_FACTOR = 1.0 + MAX_INCREASE;

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



                int xa_small = std::min(xa_last, xa);
                int xa_large = std::max(xa_last, xa);

                bool valid_acc = (
                                     // 1) movement into the same direction
                                     ((x_sign == x_last_sign)
                                      //   1a) and currently *stopping* to move
                                      && (( (xa < MIN_STEPS)
                                            && (xa_last == MIN_STEPS))
                                          // or, 1b) at least MIN_STEPS and not larger
                                          // than the allowed relative increase
                                          || ( (xa_small >= MIN_STEPS)
                                               && (xa_large <= int(xa_small * MAX_FACTOR)))))
                                     // or, has stopped to move (and only in this case,
                                     // the step count can be smaller than MIN_STEPS)
                                     || ( (xa == 0)
                                          && (xa_last < MIN_STEPS))
                                     // or, a single entry with a small number of steps,
                                     // followed by a pause or end of the table
                                     || ( (xa <= MIN_STEPS)
                                          && (xa_last == 0)
                                          && (xa_next == 0))
                                     // or, with or without a change of direction,
                                     // one step number zero and the other at
                                     // MIN_STEPS - at start or end of a movement
                                     || ((xa_small == 0)
                                         && (xa_large == MIN_STEPS))
                                     // or, a pause in movement (however not
                                     // allowed for both channels at start of
                                     // waveform)
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
            if (xa_last > MIN_STEPS)
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
        const t_wtable& waveforms, bool check_protection)
{

    LOG_CONTROL(LOG_INFO, "%18.6f : AsyncDriver: calling configMotion()\n",
                canlayer::get_realtime());

    // first, get current state of the grid
    state_summary = gateway.getGridState(grid_state);

#if (CAN_PROTOCOL_VERSION != 1 )
    // FIXME: disable checks for time-outs for now
    const unsigned long old_count_timeout = grid_state.count_timeout;
#endif

    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : configMotion(): error DE_NO_CONNECTION - no connection present\n",
                    canlayer::get_realtime());
        return DE_NO_CONNECTION;
    }

    // perform hardware protection checks unless
    // explicitly disabled.
    if (check_protection)
    {
        // check no FPUs have ongoing collisions
        // and has been initialized
        for (int i=0; i < config.num_fpus; i++)
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
                return DE_ABORTED_STATE;
            }

            if ( ! (grid_state.FPU_state[i].alpha_was_zeroed
                    && grid_state.FPU_state[i].beta_was_zeroed))
            {
                LOG_CONTROL(LOG_ERROR, "%18.6f : configMotion(): error DE_FPUS_NOT_CALIBRATED"
                            " - FPU %i is not calibrated and check_protection flag was not cleared\n",
                            canlayer::get_realtime(), i);
                return DE_FPUS_NOT_CALIBRATED;
            }
        }

        const E_DriverErrCode vwecode = validateWaveforms(waveforms,
                                        ConfigureMotionCommand::MIN_STEPCOUNT,
                                        ConfigureMotionCommand::MAX_STEPCOUNT,
                                        ConfigureMotionCommand::MAX_NUM_SECTIONS,
                                        ConfigureMotionCommand::MAX_REL_INCREASE);
        if (vwecode != DE_OK)
        {
            return vwecode;
        }
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
                                         last_entry);

                // send the command (the actual sending happens
                // in the TX thread in the background).
                unique_ptr<I_CAN_Command> cmd(can_command.release());
                alpha_cur[fpu_id] += step.alpha_steps;
                beta_cur[fpu_id] += step.beta_steps;

                LOG_CONTROL(LOG_VERBOSE, "%18.6f : configMotion(): sending wtable section %i, fpu # %i "
                            "= (%+4i, %+4i) steps --> pos (%7.3f, %7.3f) degree)\n",
                            canlayer::get_realtime(),
                            step_index, fpu_id, step.alpha_steps, step.beta_steps,
                            alpha_cur[fpu_id] / STEPS_PER_DEGREE_ALPHA,
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
        LOG_CONTROL(LOG_GRIDSTATE, "%18.6f : configMotion(): fpu # %i "
                    "--> pos (%5i, %5i) steps ~ (%+9.3f, %+9.3f) degree) - OK\n",
                    canlayer::get_realtime(),
                    fpu_id,
                    alpha_cur[fpu_id],
                    beta_cur[fpu_id],
                    alpha_cur[fpu_id] / STEPS_PER_DEGREE_ALPHA,
                    beta_cur[fpu_id] / STEPS_PER_DEGREE_BETA);
    }

    LOG_CONTROL(LOG_INFO, "%18.6f : configMotion(): waveforms successfully sent OK\n",
                canlayer::get_realtime());

    logGridState(config.logLevel, grid_state);

    return DE_OK;
}

E_DriverErrCode AsyncDriver::startExecuteMotionAsync(t_grid_state& grid_state,
        E_GridState& state_summary)
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
            return DE_ABORTED_STATE;
        }

    }

    /* check all FPUs in READY_* state have valid waveforms
       This check intends to make sure that even in protocol version 1,
       waveforms are not used when they have been involved
       in collision or abort. */
    for (int i=0; i < config.num_fpus; i++)
    {
        E_FPU_STATE fpu_status = grid_state.FPU_state[i].state;
        if (((fpu_status == FPST_READY_FORWARD)
                || (fpu_status == FPST_READY_BACKWARD))
                && ( ! (grid_state.FPU_state[i].waveform_valid
                        && grid_state.FPU_state[i].waveform_ready)))
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : executeMotion(): error DE_WAVEFORM_NOT_READY for FPU %i: no waveform ready\n",
                        canlayer::get_realtime(), i);
            return DE_WAVEFORM_NOT_READY;
        }
    }


    // send broadcast command to each gateway to start movement of all
    // FPUs.
    unique_ptr<ExecuteMotionCommand> can_command;

    // Get number of FPUs which will move
    int num_moving = (grid_state.Counts[FPST_READY_FORWARD]
                      + grid_state.Counts[FPST_READY_BACKWARD]);

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
    if (num_moving > 0)
    {
        // we need to send one command to each bus on each
        // gateway. The CAN bus then forwards this to all FPUs on the
        // same CAN bus. However in all other places, we address by
        // FPU id. To keep the driver-internal accounting simple, we
        // get and use a specific FPU id to which the command is sent.

        ecode = gateway.broadcastMessage<ExecuteMotionCommand>();

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

E_DriverErrCode AsyncDriver::waitExecuteMotionAsync(t_grid_state& grid_state,
        E_GridState& state_summary,
        double &max_wait_time, bool &finished)
{
    // Get number of FPUs which are moving or will move

    const t_grid_state previous_grid_state = grid_state;

    int num_moving = (grid_state.Counts[FPST_MOVING]
                      + grid_state.Counts[FPST_READY_FORWARD]
                      + grid_state.Counts[FPST_READY_BACKWARD]
                      + grid_state.count_pending
                      + grid_state.num_queued);

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

        // we include the "ready" counts too because it will
        // take a moment to pick up the command.
        num_moving = (grid_state.Counts[FPST_MOVING]
                      + grid_state.Counts[FPST_READY_FORWARD]
                      + grid_state.Counts[FPST_READY_BACKWARD]
                      + grid_state.count_pending
                      + grid_state.num_queued);
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

                return DE_ABORTED_STATE;
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
        E_GridState& state_summary)
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
    for (int i=0; i < config.num_fpus; i++)
    {
        // we exclude locked FPUs
        if (! gateway.isLocked(i) )
        {
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
    int num_pending = config.num_fpus - grid_state.Counts[FPST_LOCKED];

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

    num_pending = config.num_fpus - grid_state.Counts[FPST_LOCKED];

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
        E_GridState& state_summary)
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



    unique_ptr<GetErrorAlphaCommand> can_command1;
    for (int i=0; i < config.num_fpus; i++)
    {
        // we exclude locked FPUs
        if (! gateway.isLocked(i) )
        {
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
    int num_pending = config.num_fpus - grid_state.Counts[FPST_LOCKED];

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

    num_pending = config.num_fpus - grid_state.Counts[FPST_LOCKED];

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
        E_GridState& state_summary)
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
        if ((fpu_status == FPST_ABORTED)
                || (fpu_status == FPST_OBSTACLE_ERROR))
        {
            logGridState(config.logLevel, grid_state);
            LOG_CONTROL(LOG_ERROR, "%18.6f : repeatMotion():  error DE_UNRESOLVED_COLLISION for FPU %i,"
                        " collision needs to be resolvedfirst\n",
                        canlayer::get_realtime(), i);
            return DE_UNRESOLVED_COLLISION;
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
                || (fpu.state == FPST_READY_BACKWARD)
                || (fpu.state == FPST_RESTING))
                && fpu.waveform_valid)
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
                && fpu_state.waveform_valid)
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
        E_GridState& state_summary)
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
        if ((fpu_status == FPST_ABORTED)
                || (fpu_status == FPST_OBSTACLE_ERROR))
        {
            logGridState(config.logLevel, grid_state);

            LOG_CONTROL(LOG_ERROR, "%18.6f : reverseMotion():  error DE_UNRESOLVED_COLLISON for FPU %i\n",
                        canlayer::get_realtime(), i);
            return DE_UNRESOLVED_COLLISION;
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
                || (fpu.state == FPST_READY_BACKWARD)
                || (fpu.state == FPST_RESTING))
                && fpu.waveform_valid)
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
                && fpu_state.waveform_valid)
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
        E_GridState& state_summary)
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
    gateway.abortMotion(grid_state, state_summary);

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


E_DriverErrCode AsyncDriver::lockFPUAsync(t_grid_state& grid_state,
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

E_DriverErrCode AsyncDriver::unlockFPUAsync(t_grid_state& grid_state,
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


E_DriverErrCode AsyncDriver::pingFPUsAsync(t_grid_state& grid_state,
        E_GridState& state_summary)
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
        if (fpu_state.state != FPST_MOVING)
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
        E_GridState& state_summary)
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
    case FPST_READY_BACKWARD:
        return "READY_BACKWARD";
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
                            fpu.alpha_steps / STEPS_PER_DEGREE_ALPHA,
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


}

} // end of namespace
