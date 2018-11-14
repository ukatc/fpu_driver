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

#include "EtherCANInterface.h"


namespace mpifps
{

EtherCANInterface::EtherCANInterface(const EtherCANInterfaceConfig config_values)
    : AsyncInterface(config_values)
{

    LOG_CONTROL(LOG_INFO, "%18.6f : starting driver version '%s' for %i FPUs\n",
                ethercanif::get_realtime(), VERSION, config.num_fpus);
    LOG_CONTROL(LOG_INFO, "%18.6f : waveform_upload_pause_us = %lu\n",
                ethercanif::get_realtime(), config.waveform_upload_pause_us);
    LOG_CONTROL(LOG_INFO, "%18.6f : confirm_each_step = %s\n",
                ethercanif::get_realtime(), (config.confirm_each_step ? "True" : "False"));


}


int EtherCANInterface::getNumFPUs() const
{
    return config.num_fpus;
}


E_EtherCANErrCode EtherCANInterface::findDatum(t_grid_state& grid_state,
					       E_DATUM_SEARCH_DIRECTION * p_direction_flags,
					       E_DATUM_SELECTION arm_selection,
					       E_DATUM_TIMEOUT_FLAG timeout_flag,
					       bool count_protection,
					       t_fpuset const * const fpuset)
{

    E_EtherCANErrCode estatus = startFindDatum(grid_state,
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

E_EtherCANErrCode EtherCANInterface::startFindDatum(t_grid_state& grid_state,
						    E_DATUM_SEARCH_DIRECTION * p_direction_flags,
						    E_DATUM_SELECTION arm_selection,
						    E_DATUM_TIMEOUT_FLAG timeout_flag,
						    bool count_protection,
						    t_fpuset const * const fpuset)
{
    E_EtherCANErrCode estatus = DE_OK;
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

E_EtherCANErrCode EtherCANInterface::waitFindDatum(t_grid_state& grid_state,
						   double &max_wait_time,
						   bool &finished,
						   t_fpuset const * const fpuset)
{
    E_EtherCANErrCode estatus = DE_OK;
    E_GridState state_summary;

    finished = false;
    estatus = waitAutoFindDatumAsync(grid_state, state_summary, max_wait_time, finished, fpuset);

    if ((! finished) && (estatus == DE_OK))
    {
        estatus = DE_WAIT_TIMEOUT;
    }

    return estatus;
}



E_EtherCANErrCode EtherCANInterface::configMotion(const t_wtable& waveforms, t_grid_state& grid_state,
						  t_fpuset const &fpuset,
						  bool allow_uninitialized,
						  int ruleset_version)

{
    E_EtherCANErrCode estatus = DE_OK;
    E_GridState state_summary;
    int num_avaliable_retries = config.configmotion_max_retry_count;

    // copies the waveforms vector
    std::vector<t_waveform> cur_wtable(waveforms);

    pthread_mutex_lock(&command_creation_mutex);


    while (num_avaliable_retries > 0)
    {
        estatus = configMotionAsync(grid_state, state_summary, cur_wtable, fpuset,
                                    allow_uninitialized, ruleset_version);
	
        if ((estatus != DE_CAN_COMMAND_TIMEOUT_ERROR) && (estatus != DE_MAX_RETRIES_EXCEEDED))
        {
            // if connection is lost or command invalid, quit.
            break;
        }

        // if (state_summary == GS_READY_FORWARD)
        // {
        //     // Success: All FPUs have waveforms loaded. Loading finished.
        //     break;
        // }

        // we have most probably a time-out and need to load some
        // waveforms again. To do that we strip FPUs from
        // the configuration which have already been configured
        // succcessfully.
        //
        // FIXME: This does not yet handle collisions or
        // complicated states like a large bird nesting
        // on top of the FPUs. Probably has to be made
        // more robust for such cases.
	
	LOG_CONTROL(LOG_ERROR, "%18.6f : configMotion(): error: CAN timeout (countdown=%i), "
		    "re-loading missing waveforms\n",
		    ethercanif::get_realtime(), num_avaliable_retries);
	
	LOG_CONSOLE(LOG_ERROR, "%18.6f : configMotion(): error: CAN timeout (countdown=%i), "
		    "re-loading missing waveforms\n",
		    ethercanif::get_realtime(), num_avaliable_retries);


        // In this place, a down-counting iterator is used
        // so that erase() will not change the
        // index of the next processed item. (Looks dangerous but works
        // as defined).
        for (t_wtable::iterator it = cur_wtable.end() - 1;
	     it != cur_wtable.begin();
	     it--)
        {
            int fpu_id = it->fpu_id;
            assert(fpu_id >= 0);
            assert(fpu_id < config.num_fpus);

	    const t_fpu_state& fpu_state = grid_state.FPU_state[fpu_id];

            if ((fpu_state.state == FPST_READY_FORWARD)
		&& (fpu_state.num_waveform_segments == it->steps.size() ))
            {
                // delete entry for this FPU from table -
                // it does not need to be configured again
		LOG_CONTROL(LOG_INFO, "%18.6f : configMotion(): fpu id #%i is already at READY_FOWARD, omit it from waveform table\n",
			    ethercanif::get_realtime(), fpu_id);
		LOG_CONSOLE(LOG_INFO, "%18.6f : configMotion(): fpu id #%i is already at READY_FOWARD, omit it from waveform table\n",
			    ethercanif::get_realtime(), fpu_id);
                cur_wtable.erase(it);
            }
        }

	if (cur_wtable.size() == 0)
	{
	    break;
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

E_EtherCANErrCode EtherCANInterface::initializeGrid(t_grid_state& grid_state, t_fpuset const &fpuset)
{
    grid_state.interface_state = DS_ASSERTION_FAILED;

    return DE_FIRMWARE_UNIMPLEMENTED;
}
#pragma GCC diagnostic pop

E_EtherCANErrCode EtherCANInterface::resetFPUs(t_grid_state& grid_state, t_fpuset const &fpuset)
{
    E_EtherCANErrCode estatus = DE_OK;
    E_GridState state_summary;

    pthread_mutex_lock(&command_creation_mutex);
    estatus = resetFPUsAsync(grid_state, state_summary, fpuset);
    pthread_mutex_unlock(&command_creation_mutex);

    return estatus;
}

E_EtherCANErrCode EtherCANInterface::pingFPUs(t_grid_state& grid_state, t_fpuset const &fpuset)
{
    E_EtherCANErrCode estatus = DE_OK;
    E_GridState state_summary;

    pthread_mutex_lock(&command_creation_mutex);
    estatus = pingFPUsAsync(grid_state, state_summary, fpuset);
    pthread_mutex_unlock(&command_creation_mutex);

    return estatus;
}


E_EtherCANErrCode EtherCANInterface::startExecuteMotion(t_grid_state& grid_state, t_fpuset const &fpuset)
{
    E_EtherCANErrCode estatus = DE_OK;
    E_GridState state_summary;

    pthread_mutex_lock(&command_creation_mutex);

    estatus = startExecuteMotionAsync(grid_state, state_summary, fpuset);

    pthread_mutex_unlock(&command_creation_mutex);

    return estatus;
}

E_EtherCANErrCode EtherCANInterface::waitExecuteMotion(t_grid_state& grid_state,
						       double &max_wait_time, bool &finished, t_fpuset const &fpuset)
{
    E_EtherCANErrCode estatus = DE_OK;
    E_GridState state_summary;

    estatus = waitExecuteMotionAsync(grid_state,
                                     state_summary,
                                     max_wait_time,
                                     finished,
                                     fpuset);
    return estatus;
}

E_EtherCANErrCode EtherCANInterface::executeMotion(t_grid_state& grid_state, t_fpuset const &fpuset)
{
    E_EtherCANErrCode estatus = DE_OK;
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

E_EtherCANErrCode EtherCANInterface::repeatMotion(t_grid_state& grid_state, t_fpuset const &fpuset)
{
    E_EtherCANErrCode estatus = DE_OK;
    E_GridState state_summary;

    pthread_mutex_lock(&command_creation_mutex);

    estatus = repeatMotionAsync(grid_state, state_summary, fpuset);

    pthread_mutex_unlock(&command_creation_mutex);

    return estatus;
}

E_EtherCANErrCode EtherCANInterface::reverseMotion(t_grid_state& grid_state, t_fpuset const &fpuset)
{
    E_EtherCANErrCode estatus = DE_OK;
    E_GridState state_summary;

    pthread_mutex_lock(&command_creation_mutex);

    estatus = reverseMotionAsync(grid_state, state_summary, fpuset);

    pthread_mutex_unlock(&command_creation_mutex);

    return estatus;
}

E_EtherCANErrCode EtherCANInterface::abortMotion(t_grid_state& grid_state, t_fpuset const &fpuset)
{
    E_GridState state_summary;
    E_EtherCANErrCode estatus = DE_OK;

    // the implementation locks the command creation mutex in the waiting time.
    estatus =  abortMotionAsync(command_creation_mutex, grid_state, state_summary, fpuset);

    return estatus;
}


E_EtherCANErrCode EtherCANInterface::freeBetaCollision(int fpu_id, E_REQUEST_DIRECTION request_dir,
						       t_grid_state& grid_state)
{
    E_EtherCANErrCode estatus = DE_OK;
    E_GridState state_summary;

    pthread_mutex_lock(&command_creation_mutex);

    estatus = freeBetaCollisionAsync(fpu_id, request_dir, grid_state, state_summary);

    pthread_mutex_unlock(&command_creation_mutex);

    return estatus;
}

E_EtherCANErrCode EtherCANInterface::enableBetaCollisionProtection(t_grid_state& grid_state)
{
    E_EtherCANErrCode estatus = DE_OK;
    E_GridState state_summary;

    pthread_mutex_lock(&command_creation_mutex);

    estatus = enableBetaCollisionProtectionAsync(grid_state, state_summary);

    pthread_mutex_unlock(&command_creation_mutex);

    return estatus;
}






E_EtherCANErrCode EtherCANInterface::readRegister(uint16_t read_address, t_grid_state& grid_state, t_fpuset const &fpuset)
{
    E_GridState state_summary;
    E_EtherCANErrCode status;

    pthread_mutex_lock(&command_creation_mutex);
    status = readRegisterAsync(read_address, grid_state, state_summary, fpuset);
    pthread_mutex_unlock(&command_creation_mutex);

    return status;
}

E_EtherCANErrCode EtherCANInterface::getFirmwareVersion(t_grid_state& grid_state, t_fpuset const &fpuset)
{
    E_GridState state_summary;
    E_EtherCANErrCode status;

    pthread_mutex_lock(&command_creation_mutex);
    status = getFirmwareVersionAsync(grid_state, state_summary, fpuset);
    pthread_mutex_unlock(&command_creation_mutex);

    return status;
}




E_EtherCANErrCode EtherCANInterface::setUStepLevel(int ustep_level, t_grid_state& grid_state, t_fpuset const &fpuset)
{
    E_GridState state_summary;
    E_EtherCANErrCode status;

    pthread_mutex_lock(&command_creation_mutex);
    status = setUStepLevelAsync(ustep_level, grid_state, state_summary, fpuset);
    pthread_mutex_unlock(&command_creation_mutex);

    return status;
}

E_EtherCANErrCode EtherCANInterface::writeSerialNumber(int fpu_id, const char serial_number[],
						       t_grid_state& grid_state)
{
    E_GridState state_summary;
    E_EtherCANErrCode status;

    pthread_mutex_lock(&command_creation_mutex);
    status = writeSerialNumberAsync(fpu_id, serial_number, grid_state, state_summary);
    pthread_mutex_unlock(&command_creation_mutex);

    return status;
}

E_EtherCANErrCode EtherCANInterface::readSerialNumbers(t_grid_state& grid_state, t_fpuset const &fpuset)
{
    E_GridState state_summary;
    E_EtherCANErrCode status;

    pthread_mutex_lock(&command_creation_mutex);
    status = readSerialNumbersAsync(grid_state, state_summary, fpuset);
    pthread_mutex_unlock(&command_creation_mutex);

    return status;
}




E_EtherCANErrCode EtherCANInterface::resetStepCounter(t_grid_state& grid_state, t_fpuset const &fpuset)
{
    E_EtherCANErrCode estatus = DE_OK;
    E_GridState state_summary;

    pthread_mutex_lock(&command_creation_mutex);
    estatus = resetStepCounterAsync(grid_state, state_summary, fpuset);
    pthread_mutex_unlock(&command_creation_mutex);

    return estatus;
}
    
E_EtherCANErrCode EtherCANInterface::enableMove(int fpu_id, t_grid_state& grid_state)
{
    E_EtherCANErrCode estatus = DE_OK;
    E_GridState state_summary;

    pthread_mutex_lock(&command_creation_mutex);

    estatus = enableMoveAsync(fpu_id, grid_state, state_summary);

    pthread_mutex_unlock(&command_creation_mutex);

    return estatus;
}

E_EtherCANErrCode EtherCANInterface::lockFPU(int fpu_id, t_grid_state& grid_state)
{
    E_EtherCANErrCode estatus = DE_OK;
    E_GridState state_summary;

    pthread_mutex_lock(&command_creation_mutex);

    estatus = lockFPUAsync(fpu_id, grid_state, state_summary);

    pthread_mutex_unlock(&command_creation_mutex);

    return estatus;
}
    
E_EtherCANErrCode EtherCANInterface::unlockFPU(int fpu_id, t_grid_state& grid_state)
{
    E_EtherCANErrCode estatus = DE_OK;
    E_GridState state_summary;

    pthread_mutex_lock(&command_creation_mutex);

    estatus = unlockFPUAsync(fpu_id, grid_state, state_summary);

    pthread_mutex_unlock(&command_creation_mutex);

    return estatus;
}

E_EtherCANErrCode  EtherCANInterface::getMinFirmwareVersion(t_fpuset const &fpuset,
							    uint8_t (&min_firmware_version)[3],
							    t_grid_state& grid_state)
{
    E_EtherCANErrCode estatus = DE_OK;
    E_GridState state_summary;
    int min_firmware_fpu;

    pthread_mutex_lock(&command_creation_mutex);


    estatus = getMinFirmwareVersionAsync(fpuset,
					 min_firmware_version,
					 min_firmware_fpu,
					 grid_state,
					 state_summary);

    pthread_mutex_unlock(&command_creation_mutex);

    return estatus;
}

E_EtherCANErrCode EtherCANInterface::enableAlphaLimitProtection(t_grid_state& grid_state)
{
    E_EtherCANErrCode estatus = DE_OK;
    E_GridState state_summary;

    pthread_mutex_lock(&command_creation_mutex);

    estatus = enableAlphaLimitProtectionAsync(grid_state, state_summary);

    pthread_mutex_unlock(&command_creation_mutex);

    return estatus;
}

E_EtherCANErrCode EtherCANInterface::freeAlphaLimitBreach(int fpu_id, E_REQUEST_DIRECTION request_dir,
							  t_grid_state& grid_state)
{
    E_EtherCANErrCode estatus = DE_OK;
    E_GridState state_summary;

    pthread_mutex_lock(&command_creation_mutex);

    estatus = freeAlphaLimitBreachAsync(fpu_id, request_dir, grid_state, state_summary);

    pthread_mutex_unlock(&command_creation_mutex);

    return estatus;
}

E_EtherCANErrCode EtherCANInterface::setStepsPerSegment(int minsteps,
							int maxsteps,
							t_grid_state& grid_state,
							t_fpuset const &fpuset)
{
    E_GridState state_summary;
    E_EtherCANErrCode status;

    pthread_mutex_lock(&command_creation_mutex);
    status = setStepsPerSegmentAsync(minsteps, maxsteps, grid_state, state_summary, fpuset);
    pthread_mutex_unlock(&command_creation_mutex);

    return status;
}

// set number of 100ns clock ticks per waveform segment
E_EtherCANErrCode EtherCANInterface::setTicksPerSegment(unsigned long ticks,
							t_grid_state& grid_state,
							t_fpuset const &fpuset)
{
    E_GridState state_summary;
    E_EtherCANErrCode status;

    pthread_mutex_lock(&command_creation_mutex);
    status = setTicksPerSegmentAsync(ticks, grid_state, state_summary, fpuset);
    pthread_mutex_unlock(&command_creation_mutex);

    return status;
}

E_EtherCANErrCode EtherCANInterface::checkIntegrity(t_grid_state& grid_state,
						    t_fpuset const &fpuset)
{
    E_GridState state_summary;
    E_EtherCANErrCode status;

    pthread_mutex_lock(&command_creation_mutex);
    status = checkIntegrityAsync(grid_state, state_summary, fpuset);
    pthread_mutex_unlock(&command_creation_mutex);

    return status;
}


}
