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
// NAME CommandQueue.h
//
// This class implements a thread-safe array of FIFOs for commands to the CAN
// layer which can be queried and waited for efficiently
//
////////////////////////////////////////////////////////////////////////////////

#ifndef  COMMAND_QUEUE_H
#define COMMAND_QUEUE_H

#include "../DriverConstants.h"
#include "I_CAN_Command.h"

#include "GridDriverConfig.h"
#include "time_utils.h"
#include "CommandPool.h"

#include <memory>
#include <deque>

using std::unique_ptr;


namespace mpifps
{

namespace canlayer
{

class CommandQueue
{

public:
    enum E_QueueState
    {
        QS_OK = 1,
        QS_OUT_OF_MEMORY = 2,
        QS_MISSING_INSTANCE = 3,
    };

    const int MAX_MESSAGE_CAPACITY = MAX_NUM_POSITIONERS * MAX_SUB_COMMANDS;


    typedef int t_command_mask;

    CommandQueue(const GridDriverConfig config_values);

    // set number of active gateways for which queue is polled
    void setNumGateways(int ngws);

    ~CommandQueue() {};

    // initializes the internal data
    E_DriverErrCode initialize();


    // deinitialize, returning internal resources
    E_DriverErrCode deInitialize();

    // returns a bitmask indicating which gateway
    // has pending commands
    t_command_mask checkForCommand() const;

    // waits until at least one command is available,
    // and returns bitmask indicating which
    // gateway has pending commands. If the
    // waiting time exceeds timeout, an all-zero
    // mask is returned.
    t_command_mask waitForCommand(timespec timeout);

    // adds a CAN command to the queue for the corresponding
    // gateway
    // This command can fail if the system is out-of memory.
    E_QueueState enqueue(int gateway_id, unique_ptr<I_CAN_Command>& new_command);

    unique_ptr<I_CAN_Command> dequeue(int gateway_id);


    // This method adds an entry to the front of the
    // queue. This is intended for error recovery,
    // when a command has been dequeued but cannot
    // be sent, and we don't want to throw away
    // the command.
    E_QueueState requeue(int gateway_id, unique_ptr<I_CAN_Command> new_command);

    // this method empties all queues, flushing
    // all messages to the memorypool pool of
    // unused  objects.
    // The intended use is when an emergency stop
    // needs to be sent, and all queued messages
    // should be discarded.
    void flushToPool(CommandPool& memory_pool);

    int getNumQueuedCommands();

    void setEventDescriptor(int fd);

private:
    const GridDriverConfig config;
    int ngateways;
    mutable pthread_mutex_t queue_mutex = PTHREAD_MUTEX_INITIALIZER;
    // condition variables which is signaled on state changes
    pthread_cond_t cond_queue_append; // is initialized with monotonic clock option

    int EventDescriptorNewCommand;

    std::deque<unique_ptr<I_CAN_Command>> fifos[MAX_NUM_GATEWAYS];

};

}

}
#endif
