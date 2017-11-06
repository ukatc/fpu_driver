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

namespace mpifps
{


    
E_DriverErrCode AsyncDriver::initializeGridAsync(t_grid_state& grid_state)
{
}

E_DriverErrCode AsyncDriver::resetFPUsAsync(t_grid_state& grid_state)
{
}

E_DriverErrCode AsyncDriver::findDatumAsync(t_grid_state& grid_state)
{
    // first, get current state of the grid
    gateway.getGridState(grid_state);
    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        return grid_state.driver_state;
    }

    // check no FPUs have ongoing collisions
    for (int i=0; i < num_fpus; i++)
    {
        E_FPU_STATE fpu_status = grid_state.FPUState[i].state;
        if ((fpu_status == FPST_ABORTED)
            || (fpu_status == FPST_COLLISION_DETECTED)
            || (fpu_status == FPST_LIMIT_STOP))
        {
            return DE_UNRESOLVED_COLLISION;
        }
    }

    // make sure each FPU is just right above datum
    int num_moving = 0;
    unique_ptr<MoveDatumOff> can_command1;
    for (int i=0; i < num_fpus; i++)
    {
        t_fpu_state& fpu_state = grid_state.FPUState[i];
        if (fpu_state.state != FPST_LOCKED)
        {
            bool move_alpha_up = (fpu_state.on_alpha_datum
                                  || fpu_state.alpha_steps < 0);
            
            bool move_beta_up = (fpu_state.on_beta_datum
                                 || fpu_state.beta_steps < 0);
            if (move_alpha_up || move_beta_up)
            {
                can_command1 = gateway.provideInstance(CCMD_MOVE_DATUM_OFF);
                can_command1.parametrize(i,
                                        move_alpha_up ? 1 : 0,
                                        move_beta_up ? 1 : 0);
                // send the command (the actual sending happens
                // in the TX thread in the background).
                gateway.sendCommand(can_command1);
                
                num_moving++;                                
            }
        }
    }

    // fpus are now moving in parallel.
    // As long as any fpus need to move, wait for
    // them to finish
    while ( (num_moving > 0) && ((grid_state.driver_state == DS_CONNECTED)))
    {
        gateway.waitForState(TGT__ABOVE_DATUM,
                             grid_state);

        // refresh count of moving fpus
        num_moving = grid_state.count_pending;            
    }

    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }
       
    // check the result of the movement operation
    for (int i=0; i < num_fpus; i++)
    {
        E_FPU_STATE fpu_status = grid_state.FPUState[i].fpu_state;
        if ((fpu_status != FPST_LOCKED)
            || (fpu_status != FPST_ABOVE_DATUM))
        {
            // at least one FPU did not reach the
            // desired state due to time-out or collision.
            // Both cases are handled on a higher level,
            // we return here.
            return OK;
        }
    }
    
    // now, all fpus are moved above the datum switch.
    // move them until downward they hit the datum switch
        
    unique_ptr<MoveDatumOn> can_command2;
    for (int i=0; i < num_fpus; i++)
    {
        t_fpu_state& fpu_state = grid_state.FPUState[i];
        if (fpu_state.state != FPST_LOCKED)
        {
            // FIXME!!!: we should add a security
            // limit so that FPUs which are off position
            // are not driven into the hard stop.
            #pragma message "avoid hitting a hard stop here"
            
            bool move_alpha_down = fpu_state.alpha_steps > 0;
            
            bool move_beta_down = fpu_state.beta_steps > 0;
            
            if (move_alpha_up || move_beta_up)
            {
                can_command2 = gateway.provideInstance(CCMD_MOVE_DATUM_ON);
                can_command2.parametrize(i,
                                        move_alpha_down ? -1 : 0,
                                        move_beta_down ? -1 : 0);
                gateway.sendCommand(i, can_command2);
                num_moving++;                                
            }
        }
    }
            
    while ( (num_moving > 0) && ((grid_state.driver_state == DS_CONNECTED)))
    {
        gateway.waitForState(TGT_AT_DATUM,
                             grid_state);

        num_moving = grid_state.count_pending;                        
    }

    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }

    return DE_OK;
        
}


E_DriverErrCode AsyncDriver::configMotionAsync(t_grid_state& grid_state, const t_wtable& waveforms)
{

    // first, get current state of the grid
    gateway.getGridState(grid_state);
    // check driver is connected
    if (grid_state.driver_state != DS_CONNECTED)
    {
        return grid_state.driver_state;
    }

    // check no FPUs have ongoing collisions
    // and has been initialized
    for (int i=0; i < num_fpus; i++)
    {
        E_FPU_STATE fpu_status = grid_state.FPUState[i].state;
        if ((fpu_status == FPST_ABORTED)
            || (fpu_status == FPST_COLLISION_DETECTED)
            || (fpu_status == FPST_LIMIT_STOP))
        {
            return E_DriverErrCode::UNRESOLVED_COLLISION;
        }

        if (!grid_state.FPUState[i].isinitialised)
        {
            return E_DriverErrCode::NOT_INITIALISED;
        }
    }



    int num_loading =  waveforms.size();
    unique_ptr<ConfigureMotion> can_command;
    // loop over number of steps in the table
    const int num_steps = waveforms[0].size();
    for (int s=0; k < num_steps)
    {
        for (int i=0; i < num_fpus; i++)
        {
            t_fpu_state& fpu_state = grid_state.FPUState[i];
            if (fpu_state.state != FPST_LOCKED)
            {
                // get a command buffer
                can_command = gateway.provideInstance(CCMD_CONFIG_MOTION);

                int fpu_id = waveforms[i].fpu_id;
                t_step_pair step = waveforms[i][s];
                bool first_entry = (s == 0);
                bool last_entry = (s == (num_steps-1));
                can_command.parametrize(fpu_id, step.alpha_steps, step.beta_steps, first_entry, last_entry);

                // send the command (the actual sending happens
                // in the TX thread in the background).
                gateway.sendCommand(fpu_id, can_command);
            }
        }
    }

    // fpus are now loading data. Wait for 
    // them to finish.
    while ( (num_loading > 0) && ((grid_state.driver_state == DS_CONNECTED)))
    {
        gateway.waitForState(TGT_READY_TO_MOVE,
                             grid_state);

        // refresh count of loading fpus
        num_moving = grid_state.count_pending;            
    }

    if (grid_state.driver_state != DS_CONNECTED)
    {
        return DE_NO_CONNECTION;
    }
    

    return DE_OK;
}

E_DriverErrCode AsyncDriver::executeMotionAsync(t_grid_state& grid_state)
{
}

E_DriverErrCode AsyncDriver::repeatMotionAsync(t_grid_state& grid_state)
{
}

E_DriverErrCode AsyncDriver::reverseMotionAsync(t_grid_state& grid_state)
{
}

E_DriverErrCode AsyncDriver::abortMotionAsync(t_grid_state& grid_state)
{
}

E_DriverErrCode AsyncDriver::assignPositionsAsync(t_grid_state& grid_state)
{
}

E_DriverErrCode AsyncDriver::lockFPUAsync(t_grid_state& grid_state)
{
}

E_DriverErrCode AsyncDriver::unlockFPUAsync(t_grid_state& grid_state)
{
}

void AsyncDriver::getGridState(t_grid_state& out_state)
{
    gateway.getGridState(out_state);
}

E_GridState AsyncDriver::waitForState(E_WaitTarget target, t_grid_state& out_detailed_state)
{
    return gateway.waitForState(target, out_detailed_state);
}
    
    

} // end of namespace
