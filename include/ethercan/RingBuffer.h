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
// NAME RingBuffer.h
//
// This class implements a thread-safe array of FIFOs for commands to the ethercan
// layer which can be queried and waited for efficiently
//
////////////////////////////////////////////////////////////////////////////////

#ifndef  RINGBUFFER_H
#define RINGBUFFER_H

#include "../InterfaceConstants.h"
#include "CAN_Command.h"

#include "../EtherCANInterfaceConfig.h"
#include "time_utils.h"
#include "CommandPool.h"

#include <memory>
#include <vector>

using std::unique_ptr;


namespace mpifps
{

namespace ethercanif
{
    class RingBuffer {

    const int MAX_MESSAGE_CAPACITY = MAX_NUM_POSITIONERS * MAX_SUB_COMMANDS;

    public:
    RingBuffer() :
    capacity(MAX_MESSAGE_CAPACITY){
	buffer.resize(capacity);
	    head = 0u;
	    tail = 0u;
	}
	bool empty() const{
	    return (head == tail);
	}

	void push_back(unique_ptr<CAN_Command> &command_ptr){
	    assert(((head + 1) % capacity) != tail);
	    buffer[head] = std::move(command_ptr);
	    head = (head + 1) % capacity;
	}

	void push_front(unique_ptr<CAN_Command> &command_ptr){
	    assert(((tail - 1 + capacity) % capacity) != head);
	    buffer[tail] = std::move(command_ptr);
	    tail = (tail - 1 + capacity) % capacity;
	}

	unique_ptr<CAN_Command> pop_front(){
	    assert(! empty());
	    unique_ptr<CAN_Command> rval = std::move(buffer[tail]);
	    tail = (tail + 1) % capacity;
	    return rval;
	}

    private:
	unsigned const capacity;
	std::vector<unique_ptr<CAN_Command>> buffer;
	unsigned head;
	unsigned tail;
    };

}
}
#endif
