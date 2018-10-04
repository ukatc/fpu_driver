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
// NAME GridDriver.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#include "GridDriver.h"


namespace mpifps
{




E_DriverErrCode GridDriver::findDatum(t_grid_state& grid_state,
                                      E_DATUM_SEARCH_DIRECTION * p_direction_flags,
                                      E_DATUM_SELECTION arm_selection,
                                      E_DATUM_TIMEOUT_FLAG timeout_flag,
                                      bool count_protection,
                                      t_fpuset const * const fpuset)
{

    E_DriverErrCode estatus = startFindDatum(grid_state,
                              p_direction_flags,
                              arm_selection,
                              timeout_flag,
                              count_protection,
                              fpuset);

    if (estatus != DE_OK)
    {
        return estatus;
    }

    bool finished = false;
    double max_wait_time = -1; // waits until the CAN timeout is hit

    estatus = waitFindDatum(grid_state, max_wait_time, finished, fpuset);

    return estatus;
}

E_DriverErrCode GridDriver::startFindDatum(t_grid_state& grid_state,
        E_DATUM_SEARCH_DIRECTION * p_direction_flags,
        E_DATUM_SELECTION arm_selection,
        E_DATUM_TIMEOUT_FLAG timeout_flag,
        bool count_protection,
        t_fpuset const * const fpuset)
{
    E_DriverErrCode estatus = DE_OK;
    E_GridState state_summary;
    int num_avaliable_retries = DEFAULT_NUM_RETRIES;

    pthread_mutex_lock(&command_creation_mutex);

    while (num_avaliable_retries > 0)
    {
        // writes grid_state into member variable
        estatus = startAutoFindDatumAsync(grid_state, state_summary,
                                          p_direction_flags,
                                          arm_selection,
                                          timeout_flag,
                                          count_protection,
                                          fpuset);

        break;

#if (CAN_PROTOCOL_VERSION > 1)
#pragma message "FIXME: insert retry code here"
#endif

        num_avaliable_retries--;
    }

    pthread_mutex_unlock(&command_creation_mutex);

    return estatus;
}

E_DriverErrCode GridDriver::waitFindDatum(t_grid_state& grid_state,
        double &max_wait_time,
        bool &finished,
        t_fpuset const * const fpuset)
{
    E_DriverErrCode estatus = DE_OK;
    E_GridState state_summary;

    finished = false;
    estatus = waitAutoFindDatumAsync(grid_state, state_summary, max_wait_time, finished, fpuset);

    if ((! finished) && (estatus == DE_OK))
    {
        estatus = DE_WAIT_TIMEOUT;
    }

    return estatus;
}



E_DriverErrCode GridDriver::configMotion(const t_wtable& waveforms, t_grid_state& grid_state,
					 t_fpuset const &fpuset,
					 bool soft_protection,
					 bool allow_uninitialized,
					 int ruleset_version)

{
    E_DriverErrCode estatus = DE_OK;
    E_GridState state_summary;
    int num_avaliable_retries = DEFAULT_NUM_RETRIES;

    // copies the waveforms vector
    std::vector<t_waveform> cur_wtable(waveforms);

    pthread_mutex_lock(&command_creation_mutex);


    while (num_avaliable_retries > 0)
    {
        estatus = configMotionAsync(grid_state, state_summary, cur_wtable, fpuset,
				    soft_protection, allow_uninitialized, ruleset_version);
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
#if (CAN_PROTOCOL_VERSION > 1)
#pragma message "FIXME: insert retry code here"
#endif

        break; // the command should retry a few times when running in the instrument

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
        // index of the next processed item. (Looks shadowy but works
        // as defined).
        for (t_wtable::iterator it = cur_wtable.end() - 1;
                it != cur_wtable.begin();
                it--)
        {
            int fpu_id = it->fpu_id;
            printf("fpu id #%i is already at READY_FOWARD, erase it from waveform table\n",
                   fpu_id);
            assert(fpu_id >= 0);
            assert(fpu_id < config.num_fpus);

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

#pragma GCC diagnostic push
#if CAN_PROTOCOL_VERION > 1
#pragma GCC diagnostic warning "-Wunused-parameter"
#else
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

E_DriverErrCode GridDriver::initializeGrid(t_grid_state& grid_state, t_fpuset const &fpuset)
{
    grid_state.driver_state = DS_ASSERTION_FAILED;

    return DE_FIRMWARE_UNIMPLEMENTED;
}
#pragma GCC diagnostic pop

E_DriverErrCode GridDriver::resetFPUs(t_grid_state& grid_state, t_fpuset const &fpuset)
{
    E_DriverErrCode estatus = DE_OK;
    E_GridState state_summary;

    pthread_mutex_lock(&command_creation_mutex);
    estatus = resetFPUsAsync(grid_state, state_summary, fpuset);
    pthread_mutex_unlock(&command_creation_mutex);

    return estatus;
}

E_DriverErrCode GridDriver::pingFPUs(t_grid_state& grid_state, t_fpuset const &fpuset)
{
    E_DriverErrCode estatus = DE_OK;
    E_GridState state_summary;

    pthread_mutex_lock(&command_creation_mutex);
    estatus = pingFPUsAsync(grid_state, state_summary, fpuset);
    pthread_mutex_unlock(&command_creation_mutex);

    return estatus;
}


E_DriverErrCode GridDriver::startExecuteMotion(t_grid_state& grid_state, t_fpuset const &fpuset)
{
    E_DriverErrCode estatus = DE_OK;
    E_GridState state_summary;

    pthread_mutex_lock(&command_creation_mutex);

    estatus = startExecuteMotionAsync(grid_state, state_summary, fpuset);

    pthread_mutex_unlock(&command_creation_mutex);

    return estatus;
}

E_DriverErrCode GridDriver::waitExecuteMotion(t_grid_state& grid_state,
        double &max_wait_time, bool &finished, t_fpuset const &fpuset)
{
    E_DriverErrCode estatus = DE_OK;
    E_GridState state_summary;

    estatus = waitExecuteMotionAsync(grid_state,
                                     state_summary,
                                     max_wait_time,
                                     finished,
                                     fpuset);
    return estatus;
}

E_DriverErrCode GridDriver::executeMotion(t_grid_state& grid_state, t_fpuset const &fpuset)
{
    E_DriverErrCode estatus = DE_OK;
    E_GridState state_summary;

    pthread_mutex_lock(&command_creation_mutex);

    estatus = startExecuteMotionAsync(grid_state, state_summary, fpuset);

    pthread_mutex_unlock(&command_creation_mutex);

    if (estatus == DE_OK)
    {
        bool finished = false;
        while (!finished)
        {
            // note it is important to pass the current gridstate
            // to detect CAN timeouts
            double wait_time_sec = 0.5;
            estatus = waitExecuteMotionAsync(grid_state,
                                             state_summary,
                                             wait_time_sec,
                                             finished,
                                             fpuset);
        }
    }

    return estatus;
}

E_DriverErrCode GridDriver::repeatMotion(t_grid_state& grid_state, t_fpuset const &fpuset)
{
    E_DriverErrCode estatus = DE_OK;
    E_GridState state_summary;

    pthread_mutex_lock(&command_creation_mutex);

    estatus = repeatMotionAsync(grid_state, state_summary, fpuset);

    pthread_mutex_unlock(&command_creation_mutex);

    return estatus;
}

E_DriverErrCode GridDriver::reverseMotion(t_grid_state& grid_state, t_fpuset const &fpuset)
{
    E_DriverErrCode estatus = DE_OK;
    E_GridState state_summary;

    pthread_mutex_lock(&command_creation_mutex);

    estatus = reverseMotionAsync(grid_state, state_summary, fpuset);

    pthread_mutex_unlock(&command_creation_mutex);

    return estatus;
}

E_DriverErrCode GridDriver::abortMotion(t_grid_state& grid_state, t_fpuset const &fpuset)
{
    E_GridState state_summary;
    E_DriverErrCode estatus = DE_OK;

    // the implementation locks the command creation mutex in the waiting time.
    estatus =  abortMotionAsync(command_creation_mutex, grid_state, state_summary, fpuset);

    return estatus;
}


E_DriverErrCode GridDriver::freeBetaCollision(int fpu_id, E_REQUEST_DIRECTION request_dir,
        t_grid_state& grid_state)
{
    E_DriverErrCode estatus = DE_OK;
    E_GridState state_summary;

    pthread_mutex_lock(&command_creation_mutex);

    estatus = freeBetaCollisionAsync(fpu_id, request_dir, grid_state, state_summary);

    pthread_mutex_unlock(&command_creation_mutex);

    return estatus;
}

E_DriverErrCode GridDriver::enableBetaCollisionProtection(t_grid_state& grid_state)
{
    E_DriverErrCode estatus = DE_OK;
    E_GridState state_summary;

    pthread_mutex_lock(&command_creation_mutex);

    estatus = enableBetaCollisionProtectionAsync(grid_state, state_summary);

    pthread_mutex_unlock(&command_creation_mutex);

    return estatus;
}



E_DriverErrCode GridDriver::lockFPU(int fpu_id, t_grid_state& grid_state)
{
    grid_state.driver_state = DS_ASSERTION_FAILED;
    return DE_FIRMWARE_UNIMPLEMENTED;
    grid_state.driver_state = DS_ASSERTION_FAILED;

    E_GridState state_summary;
    return unlockFPUAsync(fpu_id, grid_state, state_summary);

}

E_DriverErrCode GridDriver::unlockFPU(int fpu_id, t_grid_state& grid_state)
{
    grid_state.driver_state = DS_ASSERTION_FAILED;
    return DE_FIRMWARE_UNIMPLEMENTED;

    E_GridState state_summary;
    return unlockFPUAsync(fpu_id, grid_state, state_summary);
}


E_DriverErrCode GridDriver::getPositions(t_grid_state& grid_state, t_fpuset const &fpuset)
{
    E_GridState state_summary;
    E_DriverErrCode status;

    pthread_mutex_lock(&command_creation_mutex);
    status = getPositionsAsync(grid_state, state_summary, fpuset);
    pthread_mutex_unlock(&command_creation_mutex);

    return status;
}


E_DriverErrCode GridDriver::readRegister(uint16_t read_address, t_grid_state& grid_state, t_fpuset const &fpuset)
{
    E_GridState state_summary;
    E_DriverErrCode status;

    pthread_mutex_lock(&command_creation_mutex);
    status = readRegisterAsync(read_address, grid_state, state_summary, fpuset);
    pthread_mutex_unlock(&command_creation_mutex);

    return status;
}

E_DriverErrCode GridDriver::getFirmwareVersion(t_grid_state& grid_state, t_fpuset const &fpuset)
{
    E_GridState state_summary;
    E_DriverErrCode status;

    pthread_mutex_lock(&command_creation_mutex);
    status = getFirmwareVersionAsync(grid_state, state_summary, fpuset);
    pthread_mutex_unlock(&command_creation_mutex);

    return status;
}



E_DriverErrCode GridDriver::getCounterDeviation(t_grid_state& grid_state, t_fpuset const &fpuset)
{
    E_GridState state_summary;
    E_DriverErrCode status;

    pthread_mutex_lock(&command_creation_mutex);
    status = getCounterDeviationAsync(grid_state, state_summary, fpuset);
    pthread_mutex_unlock(&command_creation_mutex);

    return status;
}

E_DriverErrCode GridDriver::setUStepLevel(int ustep_level, t_grid_state& grid_state, t_fpuset const &fpuset)
{
    E_GridState state_summary;
    E_DriverErrCode status;

    pthread_mutex_lock(&command_creation_mutex);
    status = setUStepLevelAsync(ustep_level, grid_state, state_summary, fpuset);
    pthread_mutex_unlock(&command_creation_mutex);

    return status;
}

E_DriverErrCode GridDriver::writeSerialNumber(int fpu_id, const char serial_number[],
        t_grid_state& grid_state)
{
    E_GridState state_summary;
    E_DriverErrCode status;

    pthread_mutex_lock(&command_creation_mutex);
    status = writeSerialNumberAsync(fpu_id, serial_number, grid_state, state_summary);
    pthread_mutex_unlock(&command_creation_mutex);

    return status;
}

E_DriverErrCode GridDriver::readSerialNumbers(t_grid_state& grid_state, t_fpuset const &fpuset)
{
    E_GridState state_summary;
    E_DriverErrCode status;

    pthread_mutex_lock(&command_creation_mutex);
    status = readSerialNumbersAsync(grid_state, state_summary, fpuset);
    pthread_mutex_unlock(&command_creation_mutex);

    return status;
}



int GridDriver::getNumFPUs() const
{
    return config.num_fpus;
}


}
