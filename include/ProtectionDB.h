// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-05-20  TODO: Put comment here
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME ProtectionDB.h
//
// TODO: Put description here
//
////////////////////////////////////////////////////////////////////////////////


#ifndef PROTECTIONDB_H
#define PROTECTIONDB_H

#include <vector>
#include <string>
#include <cstring>
#include <memory>
#include "FPUState.h"
#include "ethercan/AsyncInterface.h"
#include "lmdb.h"
#include "Interval.h"

using namespace mpifps;
using namespace mpifps::ethercanif;


/*
Probable data types (TODO: CHECK):
    - txn:           MDB_txn &txn?? See http://www.lmdb.tech/doc/group__internal.html#structMDB__txn
    - serial_number: const char serial_number[]  (from EtherCANInterface.h)
    - subkey:        const char subkey[]  (from FPUGridDriver.py)
    - interval:      double?
    - apos:          double (see FPU driver manual)
    - bpos:          double (see FPU driver manual)
    - offset:        double (see FPU driver manual)
    - val:           MDB_val *valPtr ?
    - fpu:           t_fpu_state
    - wentry:        std::vector<t_step_pair> steps ???? (see AsyncInterface.h -> t_waveform)
                     I defined as Wentry for now - see below

*/

// TODO: Move elsewhere, and rename to e.g. Waveform?
using Wentry = std::vector<t_step_pair>;

// -----------------------------------------------------------------------------

std::string protectionDB_GetDirFromLinuxEnv(bool mockup);
void testFpuCounters();
bool protectionDB_Test();

// -----------------------------------------------------------------------------

// TODO: Check if there is already another suitable class or structure type
// defined elsewhere which can be used

// TODO: Cater for handling newer/older-version counter arrays with differing
// numbers of counters

// TODO: Figure out what FpuCounterInt needs to be:
//  - Needs to be SIGNED so that the stuff in e.g.
//    _update_counters_execute_motion() works? (TBD)
//  - Just make int64_t to cover everything?
//    N.B. int32_t/uint32_t would cover up to 2 billion/4 billion counts -
//    would this be enough over the 10-year (?) operational life of MOONS?
//  - But one of the count values holds unixtime - what size is this? (but
//    could make it a separate data type from the others if required)
using FpuCounterInt = int64_t;

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

class FpuCounters
{
public:
    FpuCounters()
    {
        counters.resize((size_t)FpuCounterId::NumCounters, 0);
    }

    void zeroAll()
    {
        for (int i = 0; i < (int)FpuCounterId::NumCounters; i++)
        {
            counters[i] = 0;
        }
    }

    void setCount(FpuCounterId id, FpuCounterInt val)
    {
        if ((((int)id) >= 0) && (id < FpuCounterId::NumCounters))
        {
            counters[(int)id] = val;
        }
    }

    void addToCount(FpuCounterId id, FpuCounterInt val)
    {
        if ((((int)id) >= 0) && (id < FpuCounterId::NumCounters))
        {
            counters[(int)id] += val;
        }
    }

    FpuCounterInt getCount(FpuCounterId id)
    {
        if ((((int)id) >= 0) && (id < FpuCounterId::NumCounters))
        {
            return counters[(int)id];
        }
        return -999;    // TODO: Is this OK to indicate error?
    }

    int getNumRawBytes()
    {
        return sizeof(FpuCounterInt) * (int)FpuCounterId::NumCounters;
    }
    
    void *getRawBytesPtr() const
    {
        // Returns pointer to the raw bytes of the counter values in memory.
        // N.B. The endianness of the counter values will be platform-dependent,
        // but this is OK because will only be used on Intel Linux boxes?
        // TODO: Is this the case?
        return const_cast<void *>((const void *)counters.data());
    }
    
    void populateFromRawBytes(void *raw_bytes_ptr)
    {
        // Populates the counters from raw byte data. Number of bytes provided
        // must be equal to that returned by getNumRawBytes(). 
        // Also see endianness comments in getRawBytesPtr() above.
        memcpy(counters.data(), raw_bytes_ptr, getNumRawBytes());
    }
    
private:
    std::vector<FpuCounterInt> counters;
};

// -----------------------------------------------------------------------------

enum class DbTransferType
{
    Read,
    Write
};

enum class FpuDbPositionType
{
    // NOTE: All values below (except for NumTypes) must also have an entry in
    // ProtectionDbTxn::fpuDbTransferPosition() -> position_subkeys[]
    AlphaLimit = 0,
    AlphaPos,
    BetaLimit,
    BetaPos,

    NumTypes
};

class ProtectionDbTxn
{
    // Important notes:
    //   - Use ProtectionDB::createTransaction() to create an instance of
    //     this class so that lifetime is managed by unique_ptr - do not
    //     create directly
    //   - Only create a single instance of this class at a time??
    //     (TODO: See LMDB database rules)

public:
    ProtectionDbTxn(MDB_env *protectiondb_mdb_env_ptr, bool &created_ok_ret);
    bool fpuDbTransferPosition(DbTransferType transfer_type,
                               FpuDbPositionType position_type, 
                               const char serial_number[], Interval &interval,
                               double &datum_offset);
    bool fpuDbPutCounters(const char serial_number[],
                          const FpuCounters &fpu_counters);
    // TODO: Implement the following functions or similar (adapted from the 
    // Python code)
#if 0
    bool fpuDbPutInterval(const char serial_number[], const char subkey[],
                          double interval, double offset = 0.0);
    bool fpuDbPutAlphaPosition(const char serial_number[], double apos, double aoffset);
    bool fpuDbPutBetaPosition(const char serial_number[], double bpos);
    bool fpuDbPutWaveformReversed(const char serial_number[], bool is_reversed);
    bool fpuDbPutWaveform(const char serial_number[], const Wentry &wentry);
    bool fpuDbPutBetaRetryCount(const char serial_number[], bool clockwise, int count);
    bool fpuDbPutAlphaRetryCount(const char serial_number[], bool clockwise, int count);
#endif // 0

    // Test functions
    bool fpuDbWriteRawItem(const char serial_number[], const char subkey[],
                           void *data_ptr, size_t num_bytes);
    bool fpuDbReadRawItem(const char serial_number[], const char subkey[],
                          void **data_ptr_ret, size_t &num_bytes_ret);

    ~ProtectionDbTxn();

private:
    int fpuDbPutItem(const char serial_number[], const char subkey[],
                     const MDB_val &data_val);
    int fpuDbGetItem(const char serial_number[], const char subkey[],
                     MDB_val &data_val_ret);
    MDB_val fpuDbCreateKeyVal(const char serial_number[],
                              const char subkey[]);

    MDB_env *env_ptr = nullptr;
    MDB_txn *txn_ptr = nullptr;
};

//..............................................................................

class ProtectionDB
{
public:
    bool open(const std::string &dir_str);
    
    std::unique_ptr<ProtectionDbTxn> createTransaction();
    
    ~ProtectionDB();
    
private:
    void close();

    MDB_env *mdb_env_ptr = nullptr;
};

// -----------------------------------------------------------------------------

#endif // PROTECTIONDB_H
