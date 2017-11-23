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
// NAME GridDriver.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#include "GridDriver.h"


namespace mpifps
{


E_DriverErrCode GridDriver::findDatum()
{
    E_DriverErrCode estatus = DE_OK;
    E_GridState state_summary;
    int num_avaliable_retries = DEFAULT_NUM_RETRIES;

    pthread_mutex_lock(&command_creation_mutex);

    while (num_avaliable_retries > 0)
    {
        // writes grid_state into member variable
        estatus = findDatumAsync(grid_state, state_summary);

        if (estatus != DE_OK)
        {
            break;
        }

        num_avaliable_retries--;
    }

    pthread_mutex_unlock(&command_creation_mutex);

    return estatus;
}




E_DriverErrCode GridDriver::configMotion(const t_wtable& waveforms)

{
    E_DriverErrCode estatus = DE_OK;
    E_GridState state_summary;
    int num_avaliable_retries = DEFAULT_NUM_RETRIES;

    // copies the waveforms vector
    std::vector<t_waveform> cur_wtable(waveforms);

    pthread_mutex_lock(&command_creation_mutex);


    while (num_avaliable_retries > 0)
    {
        estatus = configMotionAsync(grid_state, state_summary, cur_wtable);
        if (estatus != DE_OK)
        {
            // if connection is lost or command invalid, quit.
            break;
        }

        if (state_summary == GS_READY_FORWARD)
        {
            // Success: All FPUs have waveforms loaded. Loading finished.
            break;
        }

        // we have most probably a time-out and need to load some
        // waveforms again. To do that we strip FPUs from
        // the configuration which have already been configured
        // succcessfully.
        //
        // FIXME: This does not yet handle collisions or
        // complicated states like a large bird nesting
        // on top of the FPUs. Probably has to be made
        // more robust for such cases.

        // In this place, a down-counting iterator is used
        // so that erase() will not change the
        // index of the next processed item.
        for (t_wtable::iterator it = cur_wtable.end();
                it != cur_wtable.begin();
                it--)
        {
            int fpu_id = it->fpu_id;

            if (grid_state.FPU_state[fpu_id].state == FPST_READY_FORWARD)
            {
                // delete entry for this FPU from table -
                // it does not need to be configured again
                cur_wtable.erase(it);
            }
        }

        num_avaliable_retries--;
    }

    pthread_mutex_unlock(&command_creation_mutex);

    return estatus;

}

E_DriverErrCode GridDriver::initializeGrid()
{
    return DE_OK;
}

E_DriverErrCode GridDriver::resetFPUs()
{
    return DE_OK;
}

E_DriverErrCode GridDriver::executeMotion()
{
    return DE_OK;
}

E_DriverErrCode GridDriver::repeatMotion()
{
    return DE_OK;
}

E_DriverErrCode GridDriver::reverseMotion()
{
    return DE_OK;
}

E_DriverErrCode GridDriver::abortMotion()
{
    return DE_OK;
}

E_DriverErrCode GridDriver::assignPositions()
{
    return DE_OK;
}

E_DriverErrCode GridDriver::lockFPU()
{
    return DE_OK;
}

E_DriverErrCode GridDriver::unlockFPU()
{
    return DE_OK;
}


}