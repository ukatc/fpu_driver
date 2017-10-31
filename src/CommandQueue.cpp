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
// NAME CommandQueue.h
// 
// This class implements a thread-safe array of FIFOs for commands to the CAN
// layer which can be queried and waited for efficiently
//
////////////////////////////////////////////////////////////////////////////////

#include "CommandQueue,h"
namespace mpifps {

CommandQueue::CommandQueue(int ngw)
{
    ngateways = ngw;
    for (int i= 0 ; i < ngateways; i++)
    {
        // FIXME: This can throw an exception on
        // start-up if the system is out of memory
        // - is this acceptable?
        fifo[i].resize(MAX_MESSAGE_CAPACITY);
    }

    ASSERT(condition_init_monotonic(cond_queue_append) == 0);
    
}

t_command_mask CommandQueue::checkForCommand()
{
    t_command_mask rmask = 0;
    pthread_mutex_lock(&queue_mutex);
    for (int i=0; i < ngateways; i++)
    {
        if (! fifo[i].empty() )
        {
            rmask |= 1 << i;
        }            
    }
    pthread_mutex_unlock(&queue_mutex);

    return rmask;
    
};

t_command_mask CommandQueue::waitForCommand(timespec timeout)
{

    t_command_mask rmask = 0;
    pthread_mutex_lock(&queue_mutex);
    timespec cur_time;
    get_monotonic_time(cur_time);

    timespec max_abs_time = time_add(cur_time, timeout);


    int rval = 0;
    while ((rmask == 0) and (rval != ETIMEDOUT))
    {
        for (int i=0; i < ngateways; i++)
        {
            if (! fifo[i].empty() )
            {
                rmask |= 1 << i;
            }            
        }
        
        if (rmask == 0)
        {
            // Note that, in difference to select() and poll(),
            // pthread_cond_timedwait() takes an
            // *absolute* time value as time-out.
            rval = pthread_cond_timedwait(cond_queue_append,
                                          queue_mutex,
                                          max_abs_time);
            ASSERT( rval != EINVAL);
            ASSERT(rval != EPERM);
        }
    }
    pthread_mutex_unlock(&queue_mutex);

    return rmask;

    
};

E_QUEUE_STATE CommandQueue::enqueue(int gateway_id,
                                    unique_ptr<I_CAN_Command> new_command)
{

    ASSERT(gateway_id < NUM_GATEWAYS);
    ASSERT(gateway_id >= 0);

    if (new_command == null)
    {
        return MISSING_INSTANCE;
    }

    {
        pthread_mutex_lock(&queue_mutex);
    
        fifos[gateway_id].push_back(new_command);
    
        pthread_mutex_unlock(&queue_mutex);
    }

};

unique_ptr<I_CAN_Command> CommandQueue::dequeue(int gateway_id)
{
    ASSERT(gateway_id < NUM_GATEWAYS);
    ASSERT(gateway_id >= 0);

    unique_ptr<I_CAN_Command> rval;

    {
        pthread_mutex_lock(&queue_mutex);
    
        rval = fifos[gateway_id].pop_front();
    
        pthread_mutex_unlock(&queue_mutex);
    }
    return rval;
};

// IMPORTANT NOTE: This should only be called from
// the control thread. Specifically, the memory pool
// also has a protective lock
// (it is accessed from control thread and
// TX thread),
// and flushing the CommandQueue content
// to the pool acquires that lock -
// *MAKE SURE TO NOT TRIGGER DEADLOCK*.

void CommandQueue::flushToPool(CommandPool& memory_pool)
{
    unique_ptr<I_CAN_Command> cmd;
    
    pthread_mutex_lock(&queue_mutex);

    for(int i=0; i < ngateways; i++)
    {
        while (! fifos[i].empty())
        {
            cmd = fifos[i].pop_front();
            memory_pool.recycleInstance(cmd);
        }
    }
    
    pthread_mutex_unlock(&queue_mutex);
};


}
