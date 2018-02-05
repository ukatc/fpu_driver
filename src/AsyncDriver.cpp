// -*- mode: c++ -*-
////////////////////////////////////////////////////////////////////////////////
// ESO - VLT Project
//
// Copyright 2017 E.S.O,
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// Pablo Gutierrez 2017-07-22  created CAN client sample
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client sample
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME AsyncDriver.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#include "canlayer/AsyncDriver.h"
#include "canlayer/GatewayDriver.h"

// we need to include the individual CAN commands
// as they are parametrized here.
#include "canlayer/commands/AbortMotionCommand.h"
#include "canlayer/commands/ConfigureMotionCommand.h"
#include "canlayer/commands/EnableBetaCollisionProtectionCommand.h"
#include "canlayer/commands/ExecuteMotionCommand.h"
#include "canlayer/commands/ReverseMotionCommand.h"
#include "canlayer/commands/FindDatumCommand.h"
#include "canlayer/commands/FreeBetaCollisionCommand.h"
#include "canlayer/commands/GetStepsAlphaCommand.h"
#include "canlayer/commands/GetStepsBetaCommand.h"
#include "canlayer/commands/PingFPUCommand.h"
#include "canlayer/commands/ResetFPUCommand.h"
#include "canlayer/time_utils.h"

#include <cassert>
#include <unistd.h>

#ifdef DEBUG
#include <stdio.h>
#endif

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
        return DE_DRIVER_ALREADY_INITIALIZED;

    case DS_ASSERTION_FAILED:
    default:
        return DE_ASSERTION_FAILED;
    }
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
        return DE_DRIVER_NOT_INITIALIZED;
        
    case DS_CONNECTED:
        return DE_DRIVER_STILL_CONNECTED;

    default:
        return DE_ASSERTION_FAILED;
    };
    
    return gateway.deInitialize();
}

E_DriverErrCode AsyncDriver::connect(const int ngateways, const t_gateway_address gateway_addresses[])
{
    switch (gateway.getDriverState())
    {
    case DS_UNINITIALIZED:
        return DE_DRIVER_NOT_INITIALIZED;
        
    case DS_UNCONNECTED:
        break;
        
    case DS_CONNECTED:
        return DE_DRIVER_ALREADY_CONNECTED;

    case DS_ASSERTION_FAILED:
    default:
        return DE_ASSERTION_FAILED;
    }

    E_DriverErrCode err_code =  gateway.connect(ngateways, gateway_addresses);
    if (err_code == DE_OK)
    {
        num_gateways = ngateways;
    }

    return err_code;
}

E_DriverErrCode AsyncDriver::disconnect()
{
    
    switch (gateway.getDriverState())
    {
    case DS_UNINITIALIZED:
        return DE_DRIVER_NOT_INITIALIZED;
        
    case DS_UNCONNECTED:
        return DE_NO_CONNECTION;
        
    case DS_ASSERTION_FAILED:
    case DS_CONNECTED:
    default:
        break;
    }

    E_DriverErrCode err_code = gateway.disconnect();

    if (err_code == DE_OK)
    {
        num_gateways = 0;
    }

    return err_code;

}



E_DriverErrCode AsyncDriver::initializeGridAsync(t_grid_state& grid_state,
        E_GridState& state_summary)
{
    if (gateway.getDriverState() != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }    

    return DE_OK;
}

E_DriverErrCode AsyncDriver::resetFPUsAsync(t_grid_state& grid_state,
        E_GridState& state_summary)
{
    
    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);
    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

    // make sure no FPU is moving or finding datum
    bool resetok=true;
    for (int i=0; i < num_fpus; i++)
    {
        t_fpu_state& fpu_state = grid_state.FPU_state[i];
        // we exclude moving FPUs, but include FPUs which are
        // searching datum. (FIXME: double-check that).
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
        return DE_STILL_BUSY;
    }


    unique_ptr<ResetFPUCommand> can_command;
    for (int i=0; i < num_fpus; i++)
    {
        bool broadcast = false;
        can_command = gateway.provideInstance<ResetFPUCommand>();
        can_command->parametrize(i, broadcast);
        unique_ptr<I_CAN_Command> cmd(can_command.release());
        gateway.sendCommand(i, cmd);
    }
    
    int cnt_pending = num_fpus;

    while ( (cnt_pending > 0) && ((grid_state.driver_state == DS_CONNECTED)))
    {
        double max_wait_time = 0.0;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        cnt_pending = (grid_state.count_pending + grid_state.num_queued);
    }

    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

    return DE_OK;

}

E_DriverErrCode AsyncDriver::startAutoFindDatumAsync(t_grid_state& grid_state,
        E_GridState& state_summary)
{
    
    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);
    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

    // check no FPUs have ongoing collisions
    for (int i=0; i < num_fpus; i++)
    {
        E_FPU_STATE fpu_status = grid_state.FPU_state[i].state;
        if (fpu_status == FPST_OBSTACLE_ERROR)
        {
            return DE_UNRESOLVED_COLLISION;
        }
        if (fpu_status == FPST_ABORTED)
        {
            return DE_ABORTED_STATE;
        }
    }


    // All fpus which are allowed to move, are moved automatically
    // until they hit the datum switch.

//    int num_moving = 0;
    unique_ptr<FindDatumCommand> can_command;
    for (int i=0; i < num_fpus; i++)
    {
        t_fpu_state& fpu_state = grid_state.FPU_state[i];
        if ( (fpu_state.state != FPST_UNINITIALIZED)
            || (fpu_state.state != FPST_AT_DATUM)
            || (fpu_state.state != FPST_READY_FORWARD)
            || (fpu_state.state != FPST_READY_BACKWARD)
             || (fpu_state.state != FPST_RESTING))

        {
            // FIXME!!!: For production, we might better add a
            // security limit so that FPUs which are far off position
            // are not driven into the hard stop.
#pragma message "avoid hitting a hard stop here"

            bool broadcast = false;
            can_command = gateway.provideInstance<FindDatumCommand>();
#if (CAN_PROTOCOL_VERSION == 1)
            can_command->parametrize(i, broadcast);
#else
            bool auto_datum = true;
            bool clockwise_first = false; // only relevant for non-auto
            can_command->parametrize(i, broadcast, auto_datum, clockwise_first);
#endif
            unique_ptr<I_CAN_Command> cmd(can_command.release());
            gateway.sendCommand(i, cmd);
//            num_moving++;
            
        }
    }


    return DE_OK;

}

E_DriverErrCode AsyncDriver::waitAutoFindDatumAsync(t_grid_state& grid_state,
                                                    E_GridState& state_summary,
                                                    double max_wait_time, bool &finished)
{
    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }


    
    bool cancelled = false;
    

    state_summary = gateway.waitForState(TGT_NO_MORE_MOVING,
                                         grid_state, max_wait_time, cancelled);

    int num_moving = (grid_state.Counts[FPST_DATUM_SEARCH]
                  + grid_state.count_pending
                  + grid_state.num_queued);
#ifdef DEBUG2
    printf("Counts[DATUM_SEARCH]=%i, count_pending=%i, num_queued=%i, num_moving = %i\n, %s\n",
           grid_state.Counts[FPST_DATUM_SEARCH],
           grid_state.count_pending, grid_state.num_queued,
           num_moving, (num_moving > 0) ? "retry" : "finished");
#endif        

    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

    if (state_summary == GS_COLLISION)
    {
        printf("collision detected, aborting datum search");
        return DE_NEW_COLLISION;
    }

    for (int i=0; i < num_fpus; i++)
    {
        E_FPU_STATE fpu_status = grid_state.FPU_state[i].state;
        
        if (fpu_status == FPST_OBSTACLE_ERROR)
        {
            return DE_NEW_COLLISION;
        }
        if (fpu_status == FPST_ABORTED)
        {
            return DE_ABORTED_STATE;
        }

    }
    
    
    finished = (num_moving == 0) && (! cancelled);

    if (finished)
    {
        return DE_OK;
    }
    else
    {
        return DE_COMMAND_TIMEOUT;
    }
    

}



E_DriverErrCode AsyncDriver::configMotionAsync(t_grid_state& grid_state,
        E_GridState& state_summary,
        const t_wtable& waveforms)
{

    // first, get current state of the grid
    state_summary = gateway.getGridState(grid_state);
    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

    // check no FPUs have ongoing collisions
    // and has been initialized
    for (int i=0; i < num_fpus; i++)
    {
        E_FPU_STATE fpu_status = grid_state.FPU_state[i].state;
        if ((fpu_status == FPST_ABORTED)
                || (fpu_status == FPST_OBSTACLE_ERROR))
        {
            return DE_UNRESOLVED_COLLISION;
        }

        if (!grid_state.FPU_state[i].was_zeroed)
        {
            return DE_DRIVER_NOT_INITIALIZED;
        }
    }



    unique_ptr<ConfigureMotionCommand> can_command;
    // loop over number of steps in the table
    const int num_steps = waveforms[0].steps.size();
    int step_index = 0;
    int retry_downcount = 5;
    while (step_index < num_steps)
    {
        int num_loading =  waveforms.size();
        for (int fpu_index=0; fpu_index < num_loading; fpu_index++)
        {
            int fpu_id = waveforms[fpu_index].fpu_id;
            t_fpu_state& fpu_state = grid_state.FPU_state[fpu_id];
            if (fpu_state.state != FPST_LOCKED)
            {
                // get a command buffer
                can_command = gateway.provideInstance<ConfigureMotionCommand>();

                const t_step_pair& step = waveforms[fpu_index].steps[step_index];
                const bool first_entry = (step_index == 0);
                const bool last_entry = (step_index == (num_steps-1));

                // assert precondition of 14-bit step size
                const int abs_alpha_steps = abs(step.alpha_steps);
                const bool alpha_pause = abs_alpha_steps == 0;
                const bool alpha_clockwise = (step.alpha_steps < 0);
                
                const int abs_beta_steps = abs(step.beta_steps);
                const bool beta_pause = abs_beta_steps == 0;
                const bool beta_clockwise = (step.beta_steps < 0);
                
                assert( (abs_alpha_steps >> 14) == 0);
                assert( (abs_beta_steps >> 14) == 0);
                
                can_command->parametrize(fpu_id,
                                         abs_alpha_steps,
                                         alpha_pause,
                                         alpha_clockwise,
                                         abs_beta_steps,
                                         beta_pause,
                                         beta_clockwise,
                                         first_entry,
                                         last_entry);

                // send the command (the actual sending happens
                // in the TX thread in the background).
                unique_ptr<I_CAN_Command> cmd(can_command.release());
#ifdef DEBUG2                
                printf("sending (fpu_id=%i, step_index=%i ...\n", fpu_id, step_index);
#endif                
                gateway.sendCommand(fpu_id, cmd);
            }
        }
        if ((step_index == 0) && (num_steps > 1))
        {
            /* Wait and check that all FPUs are registered in LOADING
               state.  This is needed to make sure we have later a clear
               state transition for finishing the load with the last
               flag set. */
#ifdef DEBUG2
            printf("waiting for confirmation step 0\n");
#endif
            double max_wait_time = 0.0;
            bool cancelled = false;
            state_summary = gateway.waitForState(TGT_NO_MORE_PENDING,
                                                 grid_state, max_wait_time, cancelled);
            bool do_retry = false;
            //int num_loading =  waveforms.size();
            for (int fpu_index=0; fpu_index < num_loading; fpu_index++)
            {
                int fpu_id = waveforms[fpu_index].fpu_id;
                t_fpu_state& fpu_state = grid_state.FPU_state[fpu_id];
                // we retry if an FPU which we tried to configure and is
                // not locked did not change to FPST_LOADING state.
                if ((fpu_state.state != FPST_LOADING)
                    && (fpu_state.state != FPST_LOCKED))
                {
                    if (retry_downcount <= 0)
                    {
                        return DE_MAX_RETRIES_EXCEEDED;
                    }
                    do_retry = true;
                    printf("state not confirmed for FPU #%i, retry!\n", fpu_id);
                }
            }
            if (do_retry)
            {
                // we start again with loading the first step
                retry_downcount--;
                continue;
            }
            
        }
        step_index++;
    }
#ifdef DEBUG2
    printf("ready sending! waiting....\n");
#endif    

    // fpus are now loading data.
    // Wait for fpus loading to finish, or
    // to time-out.
    /// state_summary = gateway.waitForState(TGT_READY_TO_MOVE,
    double max_wait_time = 0.0;
    bool cancelled = false;
    state_summary = gateway.waitForState(TGT_NO_MORE_PENDING,
                                         grid_state, max_wait_time, cancelled);

    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

    return DE_OK;
}

E_DriverErrCode AsyncDriver::startExecuteMotionAsync(t_grid_state& grid_state,
        E_GridState& state_summary)
{
    // first, get current state of the grid
    state_summary = gateway.getGridState(grid_state);
    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

    // check no FPUs have ongoing collisions
    for (int i=0; i < num_fpus; i++)
    {
        E_FPU_STATE fpu_status = grid_state.FPU_state[i].state;
        
        if (fpu_status == FPST_OBSTACLE_ERROR)
        {
            return DE_UNRESOLVED_COLLISION;
        }
        if (fpu_status == FPST_ABORTED)
        {
            return DE_ABORTED_STATE;
        }

    }

    /* check all FPUs in READY_* state have valid waveforms
       This check intends to make sure that even in protocol version 1,
       waveforms are not used when they have been involved
       in collision or abort. */
    for (int i=0; i < num_fpus; i++)
    {
        E_FPU_STATE fpu_status = grid_state.FPU_state[i].state;
        if (((fpu_status == FPST_READY_FORWARD)
            || (fpu_status == FPST_READY_BACKWARD))
            && ( ! (grid_state.FPU_state[i].waveform_valid
                    && grid_state.FPU_state[i].waveform_ready)))
        {
            return DE_INVALID_WAVEFORM;
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

    if (num_moving > 0)
    {
        for (int i=0; i < num_gateways; i++)
        {
            can_command = gateway.provideInstance<ExecuteMotionCommand>();
            const bool do_broadcast = true;
            can_command->parametrize(i, do_broadcast);
            unique_ptr<I_CAN_Command> cmd(can_command.release());
            gateway.broadcastCommand(i, cmd);
        }
    }
    // Give up real-time priority (this is important when the caller
    // thread later enters, for example, an endless loop).
    if (USE_REALTIME_SCHEDULING)
    {
        unset_rt_priority();
    }

    return DE_OK;

}

E_DriverErrCode AsyncDriver::waitExecuteMotionAsync(t_grid_state& grid_state,
                                               E_GridState& state_summary,
                                               double max_wait_time, bool &finished)
{
   // Get number of FPUs which are moving or will move
    
    int num_moving = (grid_state.Counts[FPST_MOVING]
                      + grid_state.Counts[FPST_READY_FORWARD]
                      + grid_state.Counts[FPST_READY_BACKWARD]
                      + grid_state.count_pending
                      + grid_state.num_queued);
    
    bool cancelled = false;
    
    if ( (num_moving > 0) 
         && (grid_state.driver_state == DS_CONNECTED))
    {
#ifdef DEBUG2
        printf("AsyncDriver::waitExecuteMotion: moving=%i, rfw=%i, rbw=%i, pend=%i, nqed=%i\n",
               grid_state.Counts[FPST_MOVING],
               grid_state.Counts[FPST_READY_FORWARD],
               grid_state.Counts[FPST_READY_BACKWARD],
               grid_state.count_pending,
               grid_state.num_queued);
        printf("AsyncDriver::waitExecuteMotion, waiting (num_moving=%i)\n", num_moving);
#endif        
        

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

#ifdef DEBUG2
    printf("AsyncDriver::waitExecuteMotion return, finished =%i, cancelled = %i, num_moving = %i\n",
               finished, cancelled, num_moving);
#endif        
    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

    
    for (int i=0; i < num_fpus; i++)
    {
        E_FPU_STATE fpu_status = grid_state.FPU_state[i].state;
        
        if (fpu_status == FPST_OBSTACLE_ERROR)
        {
            return DE_NEW_COLLISION;
        }
        if (fpu_status == FPST_ABORTED)
        {
            return DE_ABORTED_STATE;
        }

    }
    
    return DE_OK;
}





E_DriverErrCode AsyncDriver::getPositionsAsync(t_grid_state& grid_state,
                                               E_GridState& state_summary)
{
    
    // first, get current state of the grid
    state_summary = gateway.getGridState(grid_state);
    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }



    unique_ptr<GetStepsAlphaCommand> can_command1;
    for (int i=0; i < num_fpus; i++)
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
    int num_pending = num_fpus - grid_state.Counts[FPST_LOCKED];

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
        double max_wait_time = 0.0;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        // get fresh count of pending fpus.
        // The reason we add the unsent command is that
        // the Tx thread might not have had opportunity
        // to send all the commands.
        num_pending = (grid_state.count_pending + grid_state.num_queued);
#ifdef DEBUG3
//        printf("$"); fflush(stdout);
        printf("getPositions(alpha) num_pending=%i\n", num_pending);
#endif
        

    }

    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

    unique_ptr<GetStepsBetaCommand> can_command2;
    for (int i=0; i < num_fpus; i++)
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
    
    num_pending = num_fpus - grid_state.Counts[FPST_LOCKED];

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
        
        double max_wait_time = 0.0;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        // get fresh count of pending fpus.
        // The reason we add the unsent command is that
        // the Tx thread might not have had opportunity
        // to send all the commands.
        num_pending = (grid_state.count_pending
                      + grid_state.num_queued);

#ifdef DEBUG3
        printf("getPositions(beta) num_pending=%i\n", num_pending);
#endif
    }

    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

    
    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

    return DE_OK;

}


E_DriverErrCode AsyncDriver::repeatMotionAsync(t_grid_state& grid_state,
        E_GridState& state_summary)
{
    // first, get current state of the grid
    state_summary = gateway.getGridState(grid_state);
    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

    return DE_OK;
}

E_DriverErrCode AsyncDriver::reverseMotionAsync(t_grid_state& grid_state,
        E_GridState& state_summary)
{

    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);
    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

    // check no FPUs have ongoing collisions or are moving
    for (int i=0; i < num_fpus; i++)
    {
        E_FPU_STATE fpu_status = grid_state.FPU_state[i].state;
        if ((fpu_status == FPST_ABORTED)
            || (fpu_status == FPST_OBSTACLE_ERROR))
        {
            return DE_UNRESOLVED_COLLISION;
        }
    }

    for (int i=0; i < num_fpus; i++)
    {
        E_FPU_STATE fpu_status = grid_state.FPU_state[i].state;
        if (fpu_status == FPST_MOVING)
        {
            return DE_STILL_BUSY;
        }
    }
    
    /* check some FPUs in READY_* or RESTING state have valid waveforms
       This check intends to make sure that even in protocol version 1,
       waveforms are not used when they have been involved
       in collision or abort. */
    int count_movable = 0;
    for (int i=0; i < num_fpus; i++)
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
        return DE_NO_MOVABLE_FPUS;
    }


    // All fpus which are in RESTING or READY_FORWARD state get a reverseMotion message.

    int cnt_pending = 0;
    unique_ptr<ReverseMotionCommand> can_command;
    for (int i=0; i < num_fpus; i++)
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
            can_command = gateway.provideInstance<ReverseMotionCommand>();
            
            can_command->parametrize(i, broadcast);
            unique_ptr<I_CAN_Command> cmd(can_command.release());
            gateway.sendCommand(i, cmd);
            cnt_pending++;
            
        }
    }

    // wait until all generated messages have been responded to
    // or have timed out.
    while ( (cnt_pending > 0) && ((grid_state.driver_state == DS_CONNECTED)))
    {
        double max_wait_time = 0.0;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        cnt_pending = (grid_state.count_pending
                       + grid_state.num_queued);
    }

    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

    return DE_OK;
    
}

E_DriverErrCode AsyncDriver::abortMotionAsync(pthread_mutex_t & command_mutex,
                                              t_grid_state& grid_state,
        E_GridState& state_summary)
{


    // first, get current state of the grid
    state_summary = gateway.getGridState(grid_state);
    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
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
        double max_wait_time = 0.0;
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
        return DE_NO_CONNECTION;
    }

    
    return DE_OK;
    
}


E_DriverErrCode AsyncDriver::lockFPUAsync(t_grid_state& grid_state,
        E_GridState& state_summary)
{
    // first, get current state of the grid
    state_summary = gateway.getGridState(grid_state);
    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

    return DE_OK;
}

E_DriverErrCode AsyncDriver::unlockFPUAsync(t_grid_state& grid_state,
        E_GridState& state_summary)
{
    // first, get current state of the grid
    state_summary = gateway.getGridState(grid_state);
    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

    return DE_OK;
}


E_DriverErrCode AsyncDriver::pingFPUsAsync(t_grid_state& grid_state,
        E_GridState& state_summary)
{
    
    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);
    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }


    // All fpus which are not moving are pinged.

    int cnt_pending = 0;
    unique_ptr<PingFPUCommand> can_command;
    for (int i=0; i < num_fpus; i++)
    {
        t_fpu_state& fpu_state = grid_state.FPU_state[i];
        // we exclude moving FPUs, but include FPUs which are
        // searching datum. (FIXME: double-check that).
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
        double max_wait_time = 0.0;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        if (grid_state.driver_state != DS_CONNECTED)
        {
            return DE_NO_CONNECTION;
        }
        
        cnt_pending = (grid_state.count_pending
                       + grid_state.num_queued);
    }


    return DE_OK;

}


E_DriverErrCode AsyncDriver::enableBetaCollisionProtectionAsync(t_grid_state& grid_state,
                                                                E_GridState& state_summary)
{
    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);
    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }


    // make sure no FPU is moving or finding datum
    bool recoveryok=true;
    for (int i=0; i < num_fpus; i++)
    {
        t_fpu_state& fpu_state = grid_state.FPU_state[i];
        // we exclude moving FPUs, but include FPUs which are
        // searching datum. (FIXME: double-check that).
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
        return DE_STILL_BUSY;
    }


    unique_ptr<EnableBetaCollisionProtectionCommand> can_command;
    for (int i=0; i < num_fpus; i++)
    {
        bool broadcast = false;
        can_command = gateway.provideInstance<EnableBetaCollisionProtectionCommand>();
        can_command->parametrize(i, broadcast);
        unique_ptr<I_CAN_Command> cmd(can_command.release());
        gateway.sendCommand(i, cmd);
    }
    
    int cnt_pending = num_fpus;

    while ( (cnt_pending > 0) && ((grid_state.driver_state == DS_CONNECTED)))
    {
        double max_wait_time = 0.0;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        cnt_pending = (grid_state.count_pending
                       + grid_state.num_queued);
    }

    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

    return DE_OK;
}

E_DriverErrCode AsyncDriver::freeBetaCollisionAsync(t_grid_state& grid_state,
                                           E_GridState& state_summary)
{
    // first, get current state and time-out count of the grid
    state_summary = gateway.getGridState(grid_state);
    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

    // make sure no FPU is moving or finding datum
    bool recoveryok=true;
    for (int i=0; i < num_fpus; i++)
    {
        t_fpu_state& fpu_state = grid_state.FPU_state[i];
        // we exclude moving FPUs, but include FPUs which are
        // searching datum. (FIXME: double-check that).
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
        return DE_STILL_BUSY;
    }


    unique_ptr<FreeBetaCollisionCommand> can_command;
    for (int i=0; i < num_fpus; i++)
    {
        bool broadcast = false;
        can_command = gateway.provideInstance<FreeBetaCollisionCommand>();
        can_command->parametrize(i, broadcast);
        unique_ptr<I_CAN_Command> cmd(can_command.release());
        gateway.sendCommand(i, cmd);
    }
    
    int cnt_pending = num_fpus;

    while ( (cnt_pending > 0) && ((grid_state.driver_state == DS_CONNECTED)))
    {
        double max_wait_time = 0.0;
        bool cancelled = false;
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state, max_wait_time, cancelled);

        cnt_pending = (grid_state.count_pending
                       + grid_state.num_queued);
    }

    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

    return DE_OK;
}


E_GridState AsyncDriver::getGridState(t_grid_state& out_state) const
{
    return gateway.getGridState(out_state);
}

E_GridState AsyncDriver::waitForState(E_WaitTarget target,
                                      t_grid_state& out_detailed_state, double max_wait_time, bool &cancelled) const
{
    return gateway.waitForState(target, out_detailed_state, max_wait_time, cancelled);
}


}

} // end of namespace
