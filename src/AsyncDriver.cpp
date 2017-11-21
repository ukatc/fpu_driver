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

#include "AsyncDriver.h"
#include "GatewayDriver.h"

// we need to include the individual CAN commands
// as they are parametrized here.
#include "commands/ConfigureMotionCommand.h"
#include "commands/MoveDatumOnCommand.h"
#include "commands/MoveDatumOffCommand.h"
#include "commands/PingCommand.h"

namespace mpifps
{

E_DriverErrCode AsyncDriver::initializeDriver()
{
    return gateway.initialize();
}


E_DriverErrCode AsyncDriver::connect(const int ngateways, const t_gateway_address gateway_addresses[])
{
    return gateway.connect(ngateways, gateway_addresses);
}

E_DriverErrCode AsyncDriver::disconnect()
{
    return gateway.disconnect();
}



E_DriverErrCode AsyncDriver::initializeGridAsync(t_grid_state& grid_state,
        E_GridState& state_summary)
{
    return DE_OK;
}

E_DriverErrCode AsyncDriver::resetFPUsAsync(t_grid_state& grid_state,
        E_GridState& state_summary)
{
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
                can_command1 = gateway.provideInstance<MoveDatumOffCommand>(CCMD_MOVE_DATUM_OFF);
                can_command1->parametrize(i,
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
                can_command2 = gateway.provideInstance<MoveDatumOnCommand>(CCMD_MOVE_DATUM_ON);
                can_command2->parametrize(i,
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
                can_command = gateway.provideInstance<ConfigureMotionCommand>(CCMD_CONFIG_MOTION);

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
    return DE_OK;
}

E_DriverErrCode AsyncDriver::repeatMotionAsync(t_grid_state& grid_state,
        E_GridState& state_summary)
{
    return DE_OK;
}

E_DriverErrCode AsyncDriver::reverseMotionAsync(t_grid_state& grid_state,
        E_GridState& state_summary)
{
    return DE_OK;
}

E_DriverErrCode AsyncDriver::abortMotionAsync(t_grid_state& grid_state,
        E_GridState& state_summary)
{
    return DE_OK;
}

E_DriverErrCode AsyncDriver::assignPositionsAsync(t_grid_state& grid_state,
        E_GridState& state_summary)
{
    return DE_OK;
}

E_DriverErrCode AsyncDriver::lockFPUAsync(t_grid_state& grid_state,
        E_GridState& state_summary)
{
    return DE_OK;
}

E_DriverErrCode AsyncDriver::unlockFPUAsync(t_grid_state& grid_state,
        E_GridState& state_summary)
{
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



} // end of namespace
