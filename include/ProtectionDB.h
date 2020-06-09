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
#include <memory>
#include "FPUState.h"
#include "AsyncInterface.h"
#include "lmdb.h"

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
using Wentry = std::vector<AsyncInterface::t_step_pair>;

// -----------------------------------------------------------------------------

bool protectionDB_Test();

// -----------------------------------------------------------------------------

// TODO: Check if there is already another suitable class or structure type
// defined elsewhere which can be used

// TODO: Cater for handling newer/older-version counter arrays with differing
// numbers of counters

class FpuCounters
{
public:
    enum class Id
    {
        // NOTE: These indexes must stay the same for backward compatibility -
        // only add new items to end of list, and do not delete any

        // TODO: Eventually change names to camelcase with a leading capital - 
        // underscores are only used for now only so that can easily search
        // for them across this C++ and Python versions

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

    FpuCounters()
    {
        counters.resize((size_t)Id::NumCounters, 0);
    }

    void *getRawData(size_t &num_bytes_ret) const
    {
        // Returns pointer to, and number of bytes of, raw counter values in
        // memory. N.B. The endianness of the counter values will be platform-
        // dependent, but this is OK because will only be used on Intel Linux
        // boxes?
        // TODO: Is this the case?
        num_bytes_ret = sizeof(CountersUintType) * (size_t)Id::NumCounters;
        return (void *)counters.data();
    }
    
private:
    // TODO: Should this be uint64_t?
    // TODO: Note that also holds unixtime - what size is this?
    using CountersUintType = uint32_t;
  
    std::vector<CountersUintType> counters;
};

// -----------------------------------------------------------------------------

class ProtectionDB
{
public:
    bool open(const std::string &dir_str);

    //..........................................................................
    class Transaction
    {
        // ProtectionDB transaction class
        // Important notes:
        //   - Use ProtectionDB::createTransaction() to create an instance of
        //     this class so that lifetime is managed by unique_ptr - do not
        //     create directly
        //   - Only create a single instance of this class at a time??
        //     (TODO: See LMDB database rules)

    public:
        Transaction(MDB_env *protectiondb_mdb_env_ptr, bool &created_ok_ret);

        // TODO: Put all FPU-database-specific reading and writing functions here
        bool fpuDbPutCounters(const char serial_number[],
                              const FpuCounters &fpu_counters);

        // Test functions
        bool fpuDbWriteRawItem(const char serial_number[], const char subkey[],
                               void *data_ptr, size_t num_bytes);
        bool fpuDbReadRawItem(const char serial_number[], const char subkey[],
                              void **data_ptr_ret, size_t &num_bytes_ret);

        ~Transaction();

    private:
        MDB_env *mdb_env_ptr = nullptr;
        MDB_txn *txn_ptr = nullptr;
    };

    //..........................................................................
    
    std::unique_ptr<ProtectionDB::Transaction> createTransaction();
    
    ~ProtectionDB();
    
private:
    void close();

    MDB_env *mdb_env_ptr = nullptr;
};


// -----------------------------------------------------------------------------
// TODO: Old experimentation code - delete once happy with new code
#if 0
// -----------------------------------------------------------------------------

class ProtectionDB_OLD
{
public:
    // TODO: For testing only
    int doStuff();

    // TODO: Check Python's @staticmethod, @classmethod (cls), other (self) - 
    // AND why does ProtectionDB Python class use these?

    // TODO: Not sure whether the following function prototypes are good 
    // correspondences to the corresponding Python functions yet - check and
    // update as go along, and also add const qualifiers wherever appropriate


    // TODO: These two were static functions in Python, but I've changed them
    // to instance functions
    void putField(MDB_txn &txn, MDB_dbi dbi, const char serial_number[],
                  const char subkey[], MDB_val &data_val);
    void putCounters(MDB_txn &txn, MDB_dbi dbi, const char serial_number[],
                     const FpuCounters &fpu_counters);
    void putInterval(MDB_txn &txn, MDB_dbi dbi, const char serial_number[],
                     const char subkey[], double interval,
                     double offset = 0.0);

    // Instance functions
    // TODO: Check if these could just be static functions as well - do they
    // use any private class data?

    // TODO: Should getRawField() and getField() actually return MDB_val * ?    

    // TODO: Change all of the following "fpu" arguments to serial numbers
    // instead? (because the Python versions of the functions all just use
    // fpu.serial_number anyway?)
    
    bool getRawField(MDB_txn &txn, MDB_dbi dbi, const char serial_number[],
                     const char subkey[], MDB_val &mdb_val_ret);
    //MDB_val *getField(MDB_txn &txn, MDB_dbi dbi, const char serial_number[],
    //                  const char subkey[]);

    void putAlphaPosition(MDB_txn &txn, MDB_dbi dbi, const char serial_number[],
                          double apos, double aoffset);
    void putBetaPosition(MDB_txn &txn, MDB_dbi dbi, const char serial_number[],
                         double bpos);
    void storeReversed(MDB_txn &txn, MDB_dbi dbi, const char serial_number[],
                        bool is_reversed);
    void storeWaveform(MDB_txn &txn, MDB_dbi dbi, const char serial_number[],
                       const Wentry &wentry);
    void storeBretryCount(MDB_txn &txn, MDB_dbi dbi, const char serial_number[],
                          bool clockwise, int count);
    void storeAretryCount(MDB_txn &txn, MDB_dbi dbi, const char serial_number[],
                          bool clockwise, int count);

    // TODO: counter_vals: See end of _update_counters_execute_motion()?
    void putCcounters(MDB_txn &txn, MDB_dbi dbi, const char serial_number[], 
                      const FpuCounters &fpu_counters);
};

void protectionDB_Test_OLD();

// -----------------------------------------------------------------------------
#endif // 0
// -----------------------------------------------------------------------------

#endif // PROTECTIONDB_H
