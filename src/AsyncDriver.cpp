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
#include "canlayer/commands/GetStepsAlphaCommand.h"
#include "canlayer/commands/GetStepsBetaCommand.h"
#include "canlayer/commands/AutoMoveDatumCommand.h"
#include "canlayer/commands/MoveDatumOnCommand.h"
#include "canlayer/commands/MoveDatumOffCommand.h"
#include "canlayer/commands/PingCommand.h"
#include "canlayer/time_utils.h"

namespace mpifps
{

namespace canlayer
{

E_DriverErrCode AsyncDriver::initializeDriver()
{
    switch (gateway.getDriverState())
    {
    case DS_UNINITIALISED:
        break;
        
    case DS_UNCONNECTED:
    case DS_CONNECTED:
        return DE_DRIVER_ALREADY_INITIALISED;

    case DS_ASSERTION_FAILED:
    default:
        return DE_ASSERTION_FAILED;
    }
    return gateway.initialize();
}


E_DriverErrCode AsyncDriver::connect(const int ngateways, const t_gateway_address gateway_addresses[])
{
    if (gateway.getDriverState() != DS_UNCONNECTED)
    {
        return DE_DRIVER_NOT_INITIALISED;
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
    if (gateway.getDriverState() != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
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
    if (gateway.getDriverState() != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }    

    return DE_OK;
}

E_DriverErrCode AsyncDriver::autoFindDatumAsync(t_grid_state& grid_state,
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
                || (fpu_status == FPST_COLLISION_DETECTED)
                || (fpu_status == FPST_LIMIT_STOP))
        {
            return DE_UNRESOLVED_COLLISION;
        }
    }


    // All fpus are moved automatically until they hit the datum
    // switch.

    int num_moving = 0;
    unique_ptr<AutoMoveDatumCommand> can_command;
    for (int i=0; i < num_fpus; i++)
    {
        t_fpu_state& fpu_state = grid_state.FPU_state[i];
        if (fpu_state.state != FPST_LOCKED)
        {
            // FIXME!!!: we should add a security
            // limit so that FPUs which are off position
            // are not driven into the hard stop.
#pragma message "avoid hitting a hard stop here"

            bool broadcast = false;
            int adir = 0;
            int bdir = 0;
            can_command = gateway.provideInstance<AutoMoveDatumCommand>();
            can_command->parametrize(i, broadcast, adir, bdir);
            gateway.sendCommand(i, std::move(can_command));
            num_moving++;
            
        }
    }

    while ( (num_moving > 0) && ((grid_state.driver_state == DS_CONNECTED)))
    {
        state_summary = gateway.waitForState(TGT_AT_DATUM,
                                             grid_state);

        num_moving = grid_state.Counts[FPST_DATUM_SEARCH];
    }

    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

    return DE_OK;

}


E_DriverErrCode AsyncDriver::findDatumAsync(t_grid_state& grid_state,
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
                || (fpu_status == FPST_COLLISION_DETECTED)
                || (fpu_status == FPST_LIMIT_STOP))
        {
            return DE_UNRESOLVED_COLLISION;
        }
    }

    // make sure each FPU is just right above datum
    int num_moving = 0;
    unique_ptr<MoveDatumOffCommand> can_command1;
    for (int i=0; i < num_fpus; i++)
    {
        t_fpu_state& fpu_state = grid_state.FPU_state[i];
        if (fpu_state.state != FPST_LOCKED)
        {
            bool move_alpha_up = (fpu_state.on_alpha_datum
                                  || fpu_state.alpha_steps < 0);

            bool move_beta_up = (fpu_state.on_beta_datum
                                 || fpu_state.beta_steps < 0);
            if (move_alpha_up || move_beta_up)
            {
                can_command1 = gateway.provideInstance<MoveDatumOffCommand>();
                bool broadcast = false;
                can_command1->parametrize(i,
                                          broadcast,
                                          move_alpha_up ? 1 : 0,
                                          move_beta_up ? 1 : 0);
                // send the command (the actual sending happens
                // in the TX thread in the background).
                gateway.sendCommand(i, std::move(can_command1));

                num_moving++;
            }
        }
    }

    // fpus are now moving in parallel. Their status
    // goes from FPST_UNINITIALISED OR FPST_FINISHED
    // to FPST_LEAVING_DATUM, and changes to
    // FPST_ABOVE_DATUM once the command sent finishes.
    //
    // As long as any fpus need to move, wait for
    // them to finish.
    while ( (num_moving > 0) && ((grid_state.driver_state == DS_CONNECTED)))
    {
        state_summary = gateway.waitForState(TGT_ABOVE_DATUM,
                                             grid_state);

        // refresh count of moving fpus
        // This relies on that each FPU sends a response
        // when it reaches the datum, or otherwise
        // the time-out handler sets the status back
        // to FPST_UNINITIALISED.
        num_moving = grid_state.Counts[FPST_LEAVING_DATUM];
    }

    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

    // check the result of the movement operation
    for (int i=0; i < num_fpus; i++)
    {
        E_FPU_STATE fpu_status = grid_state.FPU_state[i].state;
        if ((fpu_status != FPST_LOCKED)
                || (fpu_status != FPST_ABOVE_DATUM))
        {
            // at least one FPU did not reach the
            // desired state due to time-out or collision.
            // Both cases are handled on a higher level,
            // we return here.
            return DE_OK;
        }
    }

    // now, all fpus are moved above the datum switch.
    // move them until downward they hit the datum switch

    unique_ptr<MoveDatumOnCommand> can_command2;
    for (int i=0; i < num_fpus; i++)
    {
        t_fpu_state& fpu_state = grid_state.FPU_state[i];
        if (fpu_state.state != FPST_LOCKED)
        {
            // FIXME!!!: we should add a security
            // limit so that FPUs which are off position
            // are not driven into the hard stop.
#pragma message "avoid hitting a hard stop here"

            bool move_alpha_down = fpu_state.alpha_steps > 0;

            bool move_beta_down = fpu_state.beta_steps > 0;

            if (move_alpha_down || move_beta_down)
            {
                can_command2 = gateway.provideInstance<MoveDatumOnCommand>();
                bool broadcast = false;
                can_command2->parametrize(i,
                                          broadcast,
                                          move_alpha_down ? -1 : 0,
                                          move_beta_down ? -1 : 0);
                gateway.sendCommand(i, std::move(can_command2));
                num_moving++;
            }
        }
    }

    while ( (num_moving > 0) && ((grid_state.driver_state == DS_CONNECTED)))
    {
        state_summary = gateway.waitForState(TGT_AT_DATUM,
                                             grid_state);

        num_moving = grid_state.Counts[FPST_DATUM_SEARCH];
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
                || (fpu_status == FPST_COLLISION_DETECTED)
                || (fpu_status == FPST_LIMIT_STOP))
        {
            return DE_UNRESOLVED_COLLISION;
        }

        if (!grid_state.FPU_state[i].is_initialized)
        {
            return DE_NOT_INITIALISED;
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
                can_command->parametrize(fpu_id, step.alpha_steps, step.beta_steps, first_entry, last_entry);

                // send the command (the actual sending happens
                // in the TX thread in the background).
                gateway.sendCommand(fpu_id, std::move(can_command));
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
                || (fpu_status == FPST_COLLISION_DETECTED)
                || (fpu_status == FPST_LIMIT_STOP))
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

    if (num_moving > 0)
    {
        for (int i=0; i < num_gateways; i++)
        {
            can_command = gateway.provideInstance<ExecuteMotionCommand>();
            bool do_broadcast = true;
            can_command->parametrize(i, do_broadcast);
            gateway.broadcastCommand(i, std::move(can_command));
        }
    }

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
                      + grid_state.Counts[FPST_READY_BACKWARD]);
    }

    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

    
    return DE_OK;
}



// function that checks whether all FPUs have an refreshed status
// timestamp.
bool check_all_fpus_updated(int num_fpus,
                            t_grid_state& old_grid_state,
                            t_grid_state& grid_state)
{
    bool all_updated = true;
    for(int i = 0; i < num_fpus; i++)
    {
        if (grid_state.FPU_state[i].state != FPST_LOCKED)
        {
            timespec new_timestamp = grid_state.FPU_state[i].last_updated;
            timespec old_timestamp = old_grid_state.FPU_state[i].last_updated;
            if (time_equal(old_timestamp, new_timestamp))
            {
                all_updated = false;
                break;
            }
                
        }
    }
    return all_updated;
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

    // this needs a few KB of stack space.
    t_grid_state old_grid_state = grid_state;


    unique_ptr<GetStepsAlphaCommand> can_command1;
    for (int i=0; i < num_fpus; i++)
    {
        // we exclude locked FPUs
        if (! gateway.isLocked(i) )
        {
            can_command1 = gateway.provideInstance<GetStepsAlphaCommand>();
            bool broadcast = false;
            can_command1->parametrize(i, broadcast);
            // send the command (the actual sending happens
            // in the TX thread in the background).
            gateway.sendCommand(i, std::move(can_command1));
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
        state_summary = gateway.waitForState(TGT_ANY_CHANGE,
                                             grid_state);

        // get fresh count of pending fpus
        num_pending = grid_state.count_pending;

        // check for any time-out. (Note this checks on
        // purpose for inequality, as the time-out counter
        // is unsigned and can wrap around. But in difference
        // to a signed counter, this won't cause undefined
        // behavior and is safe to use.).
        bool new_timeout = (old_grid_state.count_timeout
                            != grid_state.count_timeout);

        // we return if no more commands are pending,
        // and we either have all fpus updated (the normal case)
        // or we observed a time-out.
        // In the latter case, we return to avoid a dead-lock.
        if ((num_pending == 0)
            && ( new_timeout
                 || (check_all_fpus_updated(num_fpus,
                                            old_grid_state,
                                            grid_state))))
        {
            break;
        }
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
            bool broadcast = false;
            can_command2->parametrize(i, broadcast);
            // send the command (the actual sending happens
            // in the TX thread in the background).
            gateway.sendCommand(i, std::move(can_command2));
        }
    }
    
    num_pending = num_fpus - grid_state.Counts[FPST_LOCKED];

    while ( (num_pending > 0) && ((grid_state.driver_state == DS_CONNECTED)))
    {
        // as we do not effect any change on the grid,
        // we need to wait for any response event,
        // and filter out whether we are actually ready.
        state_summary = gateway.waitForState(TGT_ANY_CHANGE,
                                             grid_state);

        // get fresh count of pending fpus
        num_pending = grid_state.count_pending;

        // check for any time-out
        bool new_timeout = (old_grid_state.count_timeout
                            != grid_state.count_timeout);

        // we return if no more commands are pending,
        // and we either have all fpus updated (the normal case)
        // or we observed a time-out.
        // In the latter case, we return to avoid a dead-lock.
        if ((num_pending == 0)
            && ( new_timeout
                 || (check_all_fpus_updated(num_fpus,
                                            old_grid_state,
                                            grid_state))))
        {
            break;
        }
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

    return DE_OK;
}

E_DriverErrCode AsyncDriver::assignPositionsAsync(t_grid_state& grid_state,
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
