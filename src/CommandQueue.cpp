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
// NAME CommandQueue.cpp
//
// This class implements a thread-safe array of FIFOs for commands to the CAN
// layer which can be queried and waited for efficiently
//
////////////////////////////////////////////////////////////////////////////////

#include <cassert>

#include "sync_utils.h"

#include "CommandQueue.h"


namespace mpifps
{

CommandQueue::CommandQueue()
{

    assert(condition_init_monotonic(cond_queue_append) == 0);

}

CommandQueue::t_command_mask CommandQueue::checkForCommand()
{
    t_command_mask rmask = 0;
    pthread_mutex_lock(&queue_mutex);
    for (int i=0; i < ngateways; i++)
    {
        if (! fifos[i].empty() )
        {
            rmask |= 1 << i;
        }
    }
    pthread_mutex_unlock(&queue_mutex);

    return rmask;

}

CommandQueue::t_command_mask CommandQueue::waitForCommand(timespec timeout)
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
            if (! fifos[i].empty() )
            {
                rmask |= 1 << i;
            }
        }

        if (rmask == 0)
        {
            // Note that, in difference to select() and poll(),
            // pthread_cond_timedwait() takes an
            // *absolute* time value as time-out.
            rval = pthread_cond_timedwait(&cond_queue_append,
                                          &queue_mutex,
                                          &max_abs_time);
            assert( rval != EINVAL);
            assert(rval != EPERM);
        }
    }
    pthread_mutex_unlock(&queue_mutex);

    return rmask;


}

CommandQueue::E_QueueState CommandQueue::enqueue(int gateway_id,
        unique_ptr<I_CAN_Command> new_command)
{

    assert(gateway_id < MAX_NUM_GATEWAYS);
    assert(gateway_id >= 0);

    if (new_command == nullptr)
    {
        return QS_MISSING_INSTANCE;
    }

    {
        pthread_mutex_lock(&queue_mutex);

        // FIXME: This can thow bad_alloc if the
        // system is very low on memory.
        // Best fix seems to be to replace std::dequeue
        // with a fixed-size rungbuffer -
        // will be done later.
#pragma message "TODO: make CommandQueue::enqueue() exception-safe"
        fifos[gateway_id].push_back(std::move(new_command));

        pthread_mutex_unlock(&queue_mutex);
    }
    return QS_OK;

}

unique_ptr<I_CAN_Command> CommandQueue::dequeue(int gateway_id)
{
    assert(gateway_id < MAX_NUM_GATEWAYS);
    assert(gateway_id >= 0);

    unique_ptr<I_CAN_Command> rval;

    {
        pthread_mutex_lock(&queue_mutex);

        rval = std::move(fifos[gateway_id].front());
        fifos[gateway_id].pop_front();

        pthread_mutex_unlock(&queue_mutex);
    }
    return rval;
}
  

// This should be used if a command which has
// been dequeued cannot be sent, and is added
// again to the head / front of the queue.
CommandQueue::E_QueueState CommandQueue::requeue(int gateway_id,
        unique_ptr<I_CAN_Command> new_command)
{

    assert(gateway_id < MAX_NUM_GATEWAYS);
    assert(gateway_id >= 0);

    if (new_command == nullptr)
    {
        return QS_MISSING_INSTANCE;
    }

    {
        pthread_mutex_lock(&queue_mutex);

        fifos[gateway_id].push_front(std::move(new_command));

        pthread_mutex_unlock(&queue_mutex);
    }
    return QS_OK;

}


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
            cmd = std::move(fifos[i].front());
            memory_pool.recycleInstance(cmd);
            fifos[i].pop_front();
        }
    }

    pthread_mutex_unlock(&queue_mutex);
}


}
