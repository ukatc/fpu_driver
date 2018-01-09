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
#include "canlayer/commands/ConfigureMotionCommand.h"
#include "canlayer/commands/ExecuteMotionCommand.h"
#include "canlayer/commands/AbortMotionCommand.h"
#include "canlayer/commands/ResetFPUCommand.h"
#include "canlayer/commands/GetStepsAlphaCommand.h"
#include "canlayer/commands/GetStepsBetaCommand.h"
#include "canlayer/commands/FindDatumCommand.h"
#include "canlayer/commands/PingFPUCommand.h"
#include "canlayer/time_utils.h"

#include <cassert>

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
#ifdef DEBUG
    printf("driver state before connect: %i", gateway.getDriverState());
#endif
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

#ifdef DEBUG
    printf("driver state after connect: %i", gateway.getDriverState());
#endif
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
        unique_ptr<I_CAN_Command> cmd(can_command.release());
        gateway.sendCommand(i, cmd);
    }
    
    int count_pending = num_fpus;

    while ( (count_pending > 0) && ((grid_state.driver_state == DS_CONNECTED)))
    {
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state);

        count_pending = (grid_state.count_pending
                         + gateway.getNumUnsentCommands());
    }

    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

    return DE_OK;

}

E_DriverErrCode AsyncDriver::autoFindDatumAsync(t_grid_state& grid_state,
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
        if ((fpu_status == FPST_ABORTED)
                || (fpu_status == FPST_OBSTACLE_ERROR))
        {
            return DE_UNRESOLVED_COLLISION;
        }
    }


    // All fpus which are allowed to move, are moved automatically
    // until they hit the datum switch.

    int num_moving = 0;
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
            num_moving++;
            
        }
    }

    // FIXME: The "asynchronously sending commands" part and the
    // "waiting for completion" part might need to be split into two
    // twin methods, so that the final ESO driver can execute
    // asynchronously.
    while ( (num_moving > 0) && ((grid_state.driver_state == DS_CONNECTED)))
    {
        state_summary = gateway.waitForState(E_WaitTarget(TGT_AT_DATUM
                                                          | TGT_TIMEOUT),
                                             grid_state);

        num_moving = (grid_state.Counts[FPST_DATUM_SEARCH]
                      + gateway.getNumUnsentCommands());
    }

    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

    return DE_OK;

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
    for (int s=0; s < num_steps; s++)
    {
        int num_loading =  waveforms.size();
        for (int i=0; i < num_loading; i++)
        {
            int fpu_id = waveforms[i].fpu_id;
            t_fpu_state& fpu_state = grid_state.FPU_state[fpu_id];
            if (fpu_state.state != FPST_LOCKED)
            {
                // get a command buffer
                can_command = gateway.provideInstance<ConfigureMotionCommand>();

                t_step_pair step = waveforms[i].steps[s];
                bool first_entry = (s == 0);
                bool last_entry = (s == (num_steps-1));

                // assert precondition of 14-bit step size
                assert( (step.alpha_steps >> 14) == 0);
                assert( (step.beta_steps >> 14) == 0);
                
                can_command->parametrize(fpu_id,
                                         step.alpha_steps,
                                         step.alpha_pause,
                                         step.alpha_clockwise,
                                         step.beta_steps,
                                         step.beta_pause,
                                         step.beta_clockwise,
                                         first_entry, last_entry);

                // send the command (the actual sending happens
                // in the TX thread in the background).
                unique_ptr<I_CAN_Command> cmd(can_command.release());
                gateway.sendCommand(fpu_id, cmd);
            }
        }
    }

    // fpus are now loading data.
    // Wait for fpus loading to finish, or
    // to time-out.
    state_summary = gateway.waitForState(TGT_READY_TO_MOVE,
                                         grid_state);

    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

    return DE_OK;
}

E_DriverErrCode AsyncDriver::executeMotionAsync(t_grid_state& grid_state,
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
        if ((fpu_status == FPST_ABORTED)
                || (fpu_status == FPST_OBSTACLE_ERROR))
        {
            return DE_UNRESOLVED_COLLISION;
        }
    }


    // send broadcast command to each gateway to start movement of all
    // FPUs.
    unique_ptr<ExecuteMotionCommand> can_command;

    // Get number of FPUs which will move
    int num_moving = (grid_state.Counts[FPST_READY_FORWARD]
                      + grid_state.Counts[FPST_READY_BACKWARD]);

    // acquire real-time priority so that consecutive broadcasts to
    // the different gateways are really sent in the same few
    // milliseconds.  (A lag of more than 10 - 20 milliseconds, for
    // example caused by low memory conditions and swapping, could
    // otherwise lead to collisions.)
    set_rt_priority(CONTROL_PRIORITY);
    
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
    unset_rt_priority();


    // Wait until movement is finished.
    while ( (num_moving > 0)
            && ((grid_state.driver_state == DS_CONNECTED)))
    {
        state_summary = gateway.waitForState(TGT_MOVEMENT_FINISHED,
                                             grid_state);
        
        // we include the "ready" counts too because it will
        // take a moment to pick up the command.
        num_moving = (grid_state.Counts[FPST_MOVING]
                      + grid_state.Counts[FPST_READY_FORWARD]
                      + grid_state.Counts[FPST_READY_BACKWARD]
                     + gateway.getNumUnsentCommands());
    }

    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
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



#ifdef DEBUG3
            printf(" Sending GetStepsAlpha commands:");
            fflush(stdout);
#endif
    unique_ptr<GetStepsAlphaCommand> can_command1;
    for (int i=0; i < num_fpus; i++)
    {
        // we exclude locked FPUs
        if (! gateway.isLocked(i) )
        {
            can_command1 = gateway.provideInstance<GetStepsAlphaCommand>();
#ifdef DEBUG
            if (!can_command1)
            {
                printf("no AlphaCommand provided\n");
            }
#endif
            bool broadcast = false;
            can_command1->parametrize(i, broadcast);
            // send the command (the actual sending happens
            // in the TX thread in the background).
#ifdef DEBUG3
            printf(".");
#endif
            CommandQueue::E_QueueState qstate;
            unique_ptr<I_CAN_Command> cmd1(can_command1.release());
            qstate = gateway.sendCommand(i, cmd1);
#ifdef DEBUG
            if (qstate != CommandQueue::QS_OK)
            {
                printf("(Queue state : %i) ", qstate);
            }
#endif
            
        }
    }
#ifdef DEBUG
            printf("\n");
#endif

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
//        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING
//                                                          | TGT_ALL_UPDATED),
//                                             grid_state);
        state_summary = gateway.waitForState(E_WaitTarget(TGT_TIMEOUT),
                                             grid_state);

        // get fresh count of pending fpus.
        // The reason we add the unsent command is that
        // the Tx thread might not have had opportunity
        // to send all the commands.
        num_pending = (grid_state.count_pending
                       + gateway.getNumUnsentCommands());

    }

    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

#ifdef DEBUG3
            printf(" Sending GetStepsBeta commands:");
            fflush(stdout);
#endif
    unique_ptr<GetStepsBetaCommand> can_command2;
    for (int i=0; i < num_fpus; i++)
    {
        // we exclude locked FPUs
        if (! gateway.isLocked(i) )
        {
            can_command2 = gateway.provideInstance<GetStepsBetaCommand>();
#ifdef DEBUG
            if (!can_command2)
            {
                printf("no BetaCommand provided\n");
            }
#endif
            bool broadcast = false;
            can_command2->parametrize(i, broadcast);
            // send the command (the actual sending happens
            // in the TX thread in the background).
#ifdef DEBUG3
            printf(",");
#endif
            CommandQueue::E_QueueState qstate;
            unique_ptr<I_CAN_Command> cmd2(can_command2.release());
            qstate = gateway.sendCommand(i, cmd2);
#ifdef DEBUG
            if (qstate != CommandQueue::QS_OK)
            {
                printf("(Queue state : %i) ", qstate);
            }
#endif
        }
    }
#ifdef DEBUG
            printf("\n");
#endif
    
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
//        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING
//                                                          | TGT_ALL_UPDATED),
//                                             grid_state);
        state_summary = gateway.waitForState(E_WaitTarget(TGT_TIMEOUT),
                                             grid_state);

        // get fresh count of pending fpus.
        // The reason we add the unsent command is that
        // the Tx thread might not have had opportunity
        // to send all the commands.
        num_pending = (grid_state.count_pending
                       + gateway.getNumUnsentCommands());

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
    // first, get current state of the grid
    state_summary = gateway.getGridState(grid_state);
    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

    return DE_OK;
}

E_DriverErrCode AsyncDriver::abortMotionAsync(t_grid_state& grid_state,
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
    set_rt_priority(CONTROL_PRIORITY);

    gateway.abortMotion(grid_state, state_summary);
    
    // Give up real-time priority (this is important when the caller
    // thread later enters, for example, an endless loop).
    unset_rt_priority();


    // Wait until all movements are cancelled.
    int num_moving = grid_state.Counts[FPST_MOVING] + gateway.getNumUnsentCommands();
    while ( (num_moving > 0)
            && ((grid_state.driver_state == DS_CONNECTED)))
    {
        state_summary = gateway.waitForState(TGT_MOVEMENT_FINISHED,
                                             grid_state);
        
        num_moving = (grid_state.Counts[FPST_MOVING]
                     + gateway.getNumUnsentCommands());
    } 

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


E_DriverErrCode AsyncDriver::pingFPUAsync(t_grid_state& grid_state,
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

    int count_pending = 0;
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
            unique_ptr<I_CAN_Command> cmd(can_command.release());
            gateway.sendCommand(i, cmd);
            count_pending++;
            
        }
    }

    // wait until all generated ping commands have been responded to
    // or have timed out.
    while ( (count_pending > 0) && ((grid_state.driver_state == DS_CONNECTED)))
    {
        state_summary = gateway.waitForState(E_WaitTarget(TGT_NO_MORE_PENDING),
                                             grid_state);

        count_pending = (grid_state.count_pending
                         + gateway.getNumUnsentCommands());
    }

    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

    return DE_OK;

}


E_GridState AsyncDriver::getGridState(t_grid_state& out_state)
{
    return gateway.getGridState(out_state);
}

E_GridState AsyncDriver::waitForState(E_WaitTarget target,
                                      t_grid_state& out_detailed_state)
{
    return gateway.waitForState(target, out_detailed_state);
}


}

} // end of namespace
