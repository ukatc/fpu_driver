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
// NAME FPUCounters.h
//
// TODO: Put description here
//
////////////////////////////////////////////////////////////////////////////////

#ifndef FPUCOUNTERS_H
#define FPUCOUNTERS_H

#include <vector>
#include <cstring>
#include <cstdint>

// TODO: Cater for handling newer/older-version counter arrays with differing
// numbers of counters?

// TODO: Figure out what FpuCounterInt needs to be:
//  - Needs to be SIGNED so that the stuff in e.g.
//    _update_counters_execute_motion() works? (TBD)
//  - Just make int64_t to cover everything?
//    N.B. int32_t/uint32_t would cover up to 2 billion/4 billion counts -
//    would this be enough over the 10-year (?) operational life of MOONS?
//  - But one of the count values holds unixtime - what size is this? (but
//    could make it a separate data type from the others if required)
//  - Changing the FpuCounterInt type will make any FPU counter sets
//    already-stored in the protection database incompatible
using FpuCounterInt = int64_t;

//..............................................................................

enum class FpuCounterId
{
    // IMPORTANT: These indexes must stay the same for backward compatibility -
    // only add new items to end of list (but before NumCounters), and do not
    // delete any

    // TODO: Eventually change names to camelcase with a leading capital - 
    // underscores are only used for now only so that can easily search for
    // them across the C++ and Python versions

    unixtime = 0,

    // Updated upon executeMotion
    // Aborted movements are not subtracted
    total_beta_steps = 1,           // Total step count for beta arm
    total_alpha_steps = 2,          // Total step count for alpha arm
    executed_waveforms = 3,         // Number of waveform tables executed
    alpha_direction_reversals = 4,  // Number of times alpha arm movement was reversed
    beta_direction_reversals = 5,   // Number of times alpha arm movement was reversed
    sign_alpha_last_direction = 6,  // Sign of last alpha arm movement
    sign_beta_last_direction = 7,   // Sign of last alpha arm movement
    alpha_starts = 8,               // Number of times alpha arm started to move
    beta_starts = 9,                // Number of times alpha arm started to move

    // Updated upon finish of executeMotion / findDatum
    collisions = 10,
    limit_breaches = 11,
    can_timeout = 12,
    datum_timeout = 13,
    movement_timeout = 14,

    // Updated upon finish of findDatum
    datum_count = 15,
    alpha_aberration_count = 16,
    beta_aberration_count = 17,
    datum_sum_alpha_aberration = 18,   // Sum of residual count on alpha datum
    datum_sum_beta_aberration = 19,    // Sum of residual count on beta datum
    datum_sqsum_alpha_aberration = 20, // Square sum of above
    datum_sqsum_beta_aberration = 21,  // Square sum of above
    
    NumCounters     // NOTE: Must be at end
};

//==============================================================================
class FpuCounters
{
public:
    FpuCounters();
    void zeroAll();
    void setCount(FpuCounterId id, FpuCounterInt val);
    void addToCount(FpuCounterId id, FpuCounterInt val);
    FpuCounterInt getCount(FpuCounterId id);
    int getNumRawBytes();
    void *getRawBytesPtr() const;
    void populateFromRawBytes(void *raw_bytes_ptr);
    bool operator==(const FpuCounters &other);

private:
    std::vector<FpuCounterInt> counters;
};

//==============================================================================

void testFpuCounters();

//==============================================================================


#endif // FPUCOUNTERS_H
