// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-08-27  Created (expanded out of Python protectiondb.py).
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME FPUCounters.C
//
// FPU counter functionality - stores counts of various FPU events.
//
////////////////////////////////////////////////////////////////////////////////

#include <cstring>
#include "FPUCounters.h"

//==============================================================================
FpuCounters::FpuCounters()
{
    counters.resize((size_t)FpuCounterId::NumCounters, 0);
}

//------------------------------------------------------------------------------
void FpuCounters::zeroAll()
{
    for (int i = 0; i < (int)FpuCounterId::NumCounters; i++)
    {
        counters[i] = 0;
    }
}

//------------------------------------------------------------------------------
void FpuCounters::setCount(FpuCounterId id, FpuCounterInt val)
{
    if ((((int)id) >= 0) && (id < FpuCounterId::NumCounters))
    {
        counters[(int)id] = val;
    }
}

//------------------------------------------------------------------------------
void FpuCounters::addToCount(FpuCounterId id, FpuCounterInt val)
{
    if ((((int)id) >= 0) && (id < FpuCounterId::NumCounters))
    {
        counters[(int)id] += val;
    }
}

//------------------------------------------------------------------------------
FpuCounterInt FpuCounters::getCount(FpuCounterId id)
{
    if ((((int)id) >= 0) && (id < FpuCounterId::NumCounters))
    {
        return counters[(int)id];
    }
    return -999;    // TODO: Is this OK to indicate error?
}

//------------------------------------------------------------------------------
int FpuCounters::getNumRawBytes()
{
    return sizeof(FpuCounterInt) * (int)FpuCounterId::NumCounters;
}

//------------------------------------------------------------------------------
void *FpuCounters::getRawBytesPtr() const
{
    // Returns pointer to the raw bytes of the counter values in memory.
    // N.B. The endianness of the counter values will be platform-dependent,
    // but this is OK because will only be used on Intel Linux boxes?
    // TODO: Is this the case?
    return const_cast<void *>((const void *)counters.data());
}

//------------------------------------------------------------------------------
void FpuCounters::populateFromRawBytes(void *raw_bytes_ptr)
{
    // Populates the counters from raw byte data. Number of bytes provided
    // must be equal to that returned by getNumRawBytes(). 
    // Also see endianness comments in getRawBytesPtr() above.
    memcpy(counters.data(), raw_bytes_ptr, getNumRawBytes());
}

//------------------------------------------------------------------------------
bool FpuCounters::operator==(const FpuCounters &other)
{
    for (int i = 0; i < (int)FpuCounterId::NumCounters; i++)
    {
        if (counters[i] != other.counters[i])
        {
            return false;
        }
    }
    return true;
}


//==============================================================================
// Unit test function follows
//==============================================================================
void testFpuCounters()
{
    // Ad-hoc test function - single-step through and observe the counter arrays
    // and other variables etc in the debugger to check that works OK
    
    //..........................................................................
    // Perform various general tests
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

    //..........................................................................
    // Test the "==" comparison operator overload
    FpuCounters fpu_counters_1;
    FpuCounters fpu_counters_2;
    FpuCounters fpu_counters_3;
    for (int i = 0; i < (int)FpuCounterId::NumCounters; i++)
    {
        fpu_counters_1.setCount((FpuCounterId)i, i * 10);
    }
    fpu_counters_2 = fpu_counters_1;
    fpu_counters_3 = fpu_counters_1;

    int dummy = 0;
    if (fpu_counters_1 == fpu_counters_2)
    {
        dummy++;
    }

    fpu_counters_3.setCount(FpuCounterId::total_alpha_steps, 23);
    if (fpu_counters_1 == fpu_counters_3)
    {
        dummy++;
    }

    //..........................................................................
}

