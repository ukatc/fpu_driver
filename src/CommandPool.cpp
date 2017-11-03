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
// NAME FPU_CAN_driver.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

namespace mpifps
{

E_DriverErrCode CommandPool::initialize()
{
    E_DriverErrCode rval = DRIVER_NOT_INITIALISED;

    ASSERT(num_fpus > 0);
    for (int i = 0; i < NUM_CAN_COMMANDS; i++)
    {
        int capacity=0;
        const int cap_broadcast = 10;
        const int cap_individual = num_fpus * 10;
        const int cap_wform = num_fpus * MAX_SUB_COMMANDS;
        
        switch (i)
        {
            // broadcast commands
        case BROADCAST       :
            capacity = cap_broadcast;
            break;
            
            // waveform table
        case CONFIG_MOTION   :
            capacity = cap_wform;
            break;
            
            // individual commands
        case PING_FPU        :
        case MOVE_DATUM_OFF  :
        case MOVE_DATUM_ON   :
        case EXECUTE_MOTION  :
        case REPEAT_MOTION   :
        case REVERSE_MOTION  :
        case REQUEST_STATUS  :
        case REPORT_POSITIONS:
        case ASSIGN_POSITION :
        case ABORT_MOTION    :
        case UNTANGLE_FPU    :
        case CLEAR_COLLISION :
        case CHECK_INTEGRITY :
        case RESET_FPU       :
        case LOCK_UNIT       :
        case UNLOCK_UNIT     :
            capacity = cap_individual;
        default:
            // logical error
            ASSERT(false);
        }

        pthread_mutex_lock(&pool_mutex);
        pool[i].resize(capacity);
        unique_ptr<I_CAN_CMD> ptr;
        for (int c = 0; c < capacity; c++)
        {
            switch (i)
            {
            case PING_FPU        :
                ptr = new PingCommand();
                pool[i].push_back(ptr);
                break;

            default:
                // FIXME: add any missing constructors
                //ASSERT(0);
                
            }
        }
        pthread_mutex_unlock(&pool_mutex);

        return OK;

        
    }
}
        
unique_ptr<I_CAN_Command> CommandPool::provideInstance(E_CAN_COMMAND cmd_type)
{
    unique_ptr<ICAN_Command> ptr = null;

    pthread_mutex_lock(&pool_mutex);
    if (! pool[cmd_type].empty())
    {
        ptr = pool[cmd_type].pop_back():
    }
    pthread_mutex_unlock(&pool_mutex);

    return ptr;
}

void CommandPool::recycleInstance(unique_ptr<I_COMMAND>& cmdptr)
{
    pthread_mutex_lock(&pool_mutex);
    pool[cmd_type].push_back(cmdptr);
    cmd_ptr = null; 
    pthread_mutex_unlock(&pool_mutex);
}

}
