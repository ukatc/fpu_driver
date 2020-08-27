// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-08-27  TODO: Put comment here
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME FPUCounters.C
//
// TODO: Put description here
//
////////////////////////////////////////////////////////////////////////////////

#include <cstring>
#include "FPUCounters.h"


//==============================================================================
// Unit test function follows
//==============================================================================
void testFpuCounters()
{
    FpuCounters fpu_counters;
    
    for (int id = 0; id < (int)FpuCounterId::NumCounters; id++)
    {
        fpu_counters.setCount((FpuCounterId)id, ((int)id) * 10);
    }
    
    fpu_counters.zeroAll();
    
    fpu_counters.addToCount(FpuCounterId::alpha_starts, 345);
    FpuCounterInt astarts = fpu_counters.getCount(FpuCounterId::alpha_starts);
    
    fpu_counters.addToCount(FpuCounterId::datum_count, 22);
    fpu_counters.addToCount(FpuCounterId::datum_count, 0x1234567890);
    FpuCounterInt datums = fpu_counters.getCount(FpuCounterId::datum_count);
    
    FpuCounterInt dummy_array[(int)FpuCounterId::NumCounters];
    for (int i = 0; i < (int)FpuCounterId::NumCounters; i++)
    {
        dummy_array[i] = i * 100;
    }
    fpu_counters.zeroAll();
    fpu_counters.populateFromRawBytes(dummy_array);
    
    for (int i = 0; i < (int)FpuCounterId::NumCounters; i++)
    {
        dummy_array[i] = 0;
    }
    void *raw_bytes_ptr = fpu_counters.getRawBytesPtr();
    int num_raw_bytes = fpu_counters.getNumRawBytes();
    memcpy(dummy_array, raw_bytes_ptr, num_raw_bytes);
}

