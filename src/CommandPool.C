// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client sample
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME CommandPool.cpp
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#include <cassert>

#include "InterfaceConstants.h"
#include "ethercan/time_utils.h"

#include "ethercan/CommandPool.h"
// alphabetically sorted below
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


#ifdef DEBUG
#include <stdio.h>
#endif

namespace mpifps
{

namespace ethercanif
{


E_EtherCANErrCode CommandPool::initialize()
{

    assert(config.num_fpus > 0);
    pthread_mutex_lock(&pool_mutex);
    bool allocation_error = false;
    try
    {
        // This starts to count with 1 because 0 is no
        // actual command.
        for (int i = 1; i < NUM_CAN_COMMANDS; i++)
        {
            int capacity=0;
            const int cap_individual = config.num_fpus * 10;
            const int cap_wform = config.num_fpus * MAX_SUB_COMMANDS;

            switch (i)
            {

            // waveform table
            case CCMD_CONFIG_MOTION   :
                capacity = cap_wform;
                break;

            // these are broadcast commands. They require less
            // instances in normal use, but when using FPU
            // subsets, more instances are needed.

            case CCMD_EXECUTE_MOTION    :
            case CCMD_REPEAT_MOTION     :
            case CCMD_REVERSE_MOTION    :
#if CAN_PROTOCOL_VERSION > 1
            case CCMD_CHECK_INTEGRITY   :
#endif
            case CCMD_ABORT_MOTION      :

                // individual commands
#if CAN_PROTOCOL_VERSION == 1
            case CCMD_GET_ERROR_ALPHA   :
            case CCMD_GET_ERROR_BETA    :
            case CCMD_GET_STEPS_ALPHA   :
            case CCMD_GET_STEPS_BETA    :
#else
            case CCMD_LOCK_UNIT         :
            case CCMD_UNLOCK_UNIT       :
            case CCMD_GET_FIRMWARE_VERSION          :
            case CCMD_SET_TICKS_PER_SEGMENT         :
            case CCMD_SET_STEPS_PER_SEGMENT         :
            case CCMD_FREE_ALPHA_LIMIT_BREACH       :
            case CCMD_ENABLE_ALPHA_LIMIT_PROTECTION :
            case CCMD_ENABLE_MOVE       :
#endif
            case CCMD_RESET_FPU         :
            case CCMD_PING_FPU          :
            case CCMD_ENABLE_BETA_COLLISION_PROTECTION :
            case CCMD_FREE_BETA_COLLISION    :
            case CCMD_SET_USTEP_LEVEL   :
            case CCMD_FIND_DATUM        :
            case CCMD_RESET_STEPCOUNTER :
            case CCMD_READ_REGISTER     :
            case CCMD_READ_SERIAL_NUMBER   :
            case CCMD_WRITE_SERIAL_NUMBER  :
                capacity = cap_individual;
                break;

            default:
                // logical error
                LOG_CONTROL(LOG_ERROR, "fatal error: command code %i not found!\n", i);
                assert(false);
            }

            // This can throw bad_alloc during initialization
            // if the system is very low on memory.
            pool[i].reserve(capacity);
            unique_ptr<CAN_Command> ptr;
            for (int c = 0; c < capacity; c++)
            {
                switch (i)
                {

                case CCMD_LOCK_UNIT                        :
                    ptr.reset(new LockUnitCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_UNLOCK_UNIT                      :
                    ptr.reset(new UnlockUnitCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_RESET_STEPCOUNTER                :
                    ptr.reset(new ResetStepCounterCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_GET_FIRMWARE_VERSION             :
                    ptr.reset(new GetFirmwareVersionCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_CHECK_INTEGRITY                  :
                    ptr.reset(new CheckIntegrityCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_FREE_ALPHA_LIMIT_BREACH          :
                    ptr.reset(new FreeAlphaLimitBreachCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_ENABLE_ALPHA_LIMIT_PROTECTION    :
                    ptr.reset(new EnableAlphaLimitProtectionCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_SET_TICKS_PER_SEGMENT            :
                    ptr.reset(new SetTicksPerSegmentCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_SET_STEPS_PER_SEGMENT            :
                    ptr.reset(new SetStepsPerSegmentCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_ENABLE_MOVE                      :
                    ptr.reset(new EnableMoveCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_PING_FPU        :
                    ptr.reset(new PingFPUCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_CONFIG_MOTION        :
                    ptr.reset(new ConfigureMotionCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_EXECUTE_MOTION        :
                    ptr.reset(new ExecuteMotionCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_REVERSE_MOTION        :
                    ptr.reset(new ReverseMotionCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_REPEAT_MOTION        :
                    ptr.reset(new RepeatMotionCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_ABORT_MOTION        :
                    ptr.reset(new AbortMotionCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_RESET_FPU:
                    ptr.reset(new ResetFPUCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_FIND_DATUM        :
                    ptr.reset(new FindDatumCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_ENABLE_BETA_COLLISION_PROTECTION :
                    ptr.reset(new EnableBetaCollisionProtectionCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_FREE_BETA_COLLISION    :
                    ptr.reset(new FreeBetaCollisionCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_SET_USTEP_LEVEL :
                    ptr.reset(new SetUStepLevelCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_READ_REGISTER        :
                    ptr.reset(new ReadRegisterCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_READ_SERIAL_NUMBER        :
                    ptr.reset(new ReadSerialNumberCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                case CCMD_WRITE_SERIAL_NUMBER        :
                    ptr.reset(new WriteSerialNumberCommand());
                    pool[i].push_back(std::move(ptr));
                    break;

                default:
                    assert(0);

                }
            }
        }
    }
    catch (std::bad_alloc& ba)
    {

        allocation_error = true;
    }
    pthread_mutex_unlock(&pool_mutex);
    if (allocation_error)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : GridDriver::initialize() - out of memory in CommandPool::initialize()\n",
                    ethercanif::get_realtime());
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    return DE_OK;

}



E_EtherCANErrCode CommandPool::deInitialize()
{

    assert(config.num_fpus > 0);
    bool allocation_error = false;
    try
    {
        // This starts to count with 1 because 0 is no
        // actual command.
        for (int i = 1; i < NUM_CAN_COMMANDS; i++)
        {

            pool[i].clear();
            // FIXME: std::vector<>::shrink_to_fit can thow bad_alloc if the
            // system is low on memory. This should be caught.
            pool[i].shrink_to_fit();

        }
    }
    catch (std::bad_alloc& ba)
    {

        allocation_error = true;
    }
    if (allocation_error)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : GridDriver::deinitialize() : Assertion failed: out of memory\n",
                    ethercanif::get_realtime());
        return DE_ASSERTION_FAILED;
    }

    return DE_OK;

}


// Adds a used command to the pool again,
// and if any thread is waiting, notify it
// that there are command buffers available
// again.
void CommandPool::recycleInstance(unique_ptr<CAN_Command>& cmd_ptr)
{
    pthread_mutex_lock(&pool_mutex);
    E_CAN_COMMAND cmd_type = cmd_ptr->getInstanceCommandCode();
    bool was_empty = pool[cmd_type].empty();
    pool[cmd_type].push_back(std::move(cmd_ptr));
    cmd_ptr = nullptr;
    // if we just added to an empty pool, notify one waiting thread
    // that it can make progress.
    if (was_empty)
    {
        pthread_cond_signal(&cond_pool_add);
    }
    pthread_mutex_unlock(&pool_mutex);
}



}

}
