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
// NAME TimeOutList.h
//
// This class implements an ordered list if Time_out values,
// which can be safely accessed from multiple threads
//
////////////////////////////////////////////////////////////////////////////////

#include <time.h>
#include <stdio.h>
#include <string.h>		/// strerror
#include <pthread.h>
#include <unistd.h>
#include <std>
#include "DriverConstants.h"



namespace mpifps
{

    class TimeOutList
    {
      public:
       
        typedef struct
        {
            timespec val; // absolute value of next time-out
            int id; // corresponding fpu id
        } t_toentry;

        TimeOutList();

        ~TimeOutList();

        // All public methods are thread-safe.
        // Beware using internal methods without
        // locking.

        // insert new value into list
        // This is a O(1) operation
        insertTimeOut(int const id, timespec const val);

        // clear value from list
        // This is a O(1) operation
        clearTimeOut(id);
        

        // get time and  id of next time-out event
        // which limits the maximum waiting time for a response.
        // The returned time value is Linux' monotonic
        // clock. The max_time parameter is the
        // value which is returned if no time_out is pending.
        //
        // This is O(N) in the worst case but is
        // optimized toward O(1) in typical use,
        // when most entries have the same value.
        const timespec getNextTimeOut(timespec max_time);
        
        // get and remove item with minimum value
        t_toentry pop();


      private:

        const timespec MAX_TIMESPEC = {tv_sec = TIME_T_MAX,
                                       tv_nsec = 999999999};

        // get current minimum key (timeout value)
        // This method is not thread-safe!
        const timespec minKey();
        

        timespec TimeOutsByID[MAX_NUM_POSITIONERS];
        // this mutex protects the list form concurrent access
        pthread_mutex_t list_mutex = PTHREAD_MUTEX_INITIALIZER;

        timespec cached_minimum;
        int cached_minimum_multiplicity;
        int minimum_index_lbound;

    }

}
