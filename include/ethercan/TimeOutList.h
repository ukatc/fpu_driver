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
// NAME TimeOutList.h
//
// This class implements an ordered list if Time_out values,
// which can be safely accessed from multiple threads
//
////////////////////////////////////////////////////////////////////////////////

#ifndef TIMEOUT_LIST_H
#define TIMEOUT_LIST_H

#include <time.h>
#include "time_utils.h"
#include <pthread.h>

#include "../InterfaceConstants.h"




namespace mpifps
{

namespace ethercanif
{

class TimeOutList
{
public:

    typedef struct
    {
        timespec val; // absolute value of next time-out
        int id; // corresponding fpu id
    } t_toentry;

    static const timespec MAX_TIMESPEC;

    TimeOutList();

    ~TimeOutList() {};

    // All public methods are thread-safe.
    // Beware using internal methods without
    // locking.

    // insert new value into list
    // This is a O(1) operation
    void insertTimeOut(int const id, timespec const val);

    // clear value from list
    // This is a O(1) operation
    void clearTimeOut(int const id);


    // get time and id of next time-out event.  The returned time
    // value is Linux' monotonic clock. If no time_out is pending,
    // MAX_TIMESPEC is returned.
    //
    // This is O(N) in the worst case but is optimized toward O(1) in
    // typical use, when most entries have the same value.
    const timespec getNextTimeOut();

    // get and remove item with minimum value
    t_toentry pop();


private:


    // get current minimum key (timeout value)
    // This method is not thread-safe!
    const timespec minKey();

    const timespec search_min();


    timespec TimeOutsByID[MAX_NUM_POSITIONERS];
    // this mutex protects the list form concurrent access
    pthread_mutex_t list_mutex = PTHREAD_MUTEX_INITIALIZER;

    timespec cached_minimum;
    int cached_minimum_multiplicity;
    int minimum_index_lbound;

};

}

}

#endif
