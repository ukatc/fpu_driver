// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-05-20  Created (translated from Python protectiondb.py).
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME ProtectionDB.h
//
// Grid driver LMDB database interface layer for reading and writing FPU data
// items.
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
#include "FPUCounters.h"

using namespace mpifps;
using namespace mpifps::ethercanif;

//==============================================================================

// Character to separate the key/subkey parts of the overall key strings
const char fpudb_keystr_separator_char = '#';

#define SNUM_USED_CHECK_VAL    (0xabcd)

//==============================================================================

// FpuDbData: FPU data which is stored in the protection database.
struct FpuDbData
{
    FpuDbData();
    bool operator==(const FpuDbData &other);
    bool operator!=(const FpuDbData &other);
    
     // snum_used_flag: In the database, this dummy field's existence for a
     // serial number is used to indicate that the serial number is in use
    int64_t snum_used_flag;

    Interval apos;
    Interval bpos;
    bool wf_reversed;
    Interval alimits;
    Interval blimits;
    int64_t maxaretries;
    int64_t aretries_cw;
    int64_t aretries_acw;
    int64_t maxbretries;
    int64_t bretries_cw;
    int64_t bretries_acw;
    FpuCounters counters;
    // last_waveform: Contains the last FPU waveform. A zero-sized waveform
    // means that it's not currrently valid.
    // TODO: N.B. last_waveform corresponds to an individual FPU's waveform
    // in tbe Python version's UnprotectedGridDriver.last_wavetable dictionary
    t_waveform_steps last_waveform;
    
private:
    bool isSameAsOther(const FpuDbData &other);
};

//==============================================================================

enum class DbTransferType
{
    Read,
    Write
};

enum class FpuDbPositionType
{
    AlphaLimits = 0,
    AlphaPos,
    BetaLimits,
    BetaPos,

    NumTypes
};

enum class FpuDbIntValType
{
    SnumUsedFlag = 0,
    FreeAlphaRetries,
    AlphaRetries_CW,
    AlphaRetries_ACW,
    FreeBetaRetries,
    BetaRetries_CW,
    BetaRetries_ACW,

    NumTypes
};

//..............................................................................

class ProtectionDbTxn
{
    // Important notes:
    //   - Use ProtectionDB::createTransaction() to create an instance of
    //     this class so that lifetime is managed by unique_ptr - do not
    //     create directly
    //   - Only create a single instance of this class at a time??
    //     (TODO: See LMDB database rules)

    // Declare test class as friend so that it can access protected/private
    // member variables and functions
    friend class ProtectionDBTester;

public:
    ProtectionDbTxn(MDB_env *protectiondb_mdb_env_ptr, bool &created_ok_ret);

    bool fpuDbTransferFpu(DbTransferType transfer_type,
                          const char serial_number[],
                          FpuDbData &fpu_db_data);

    bool fpuDbTransferPosition(DbTransferType transfer_type,
                               FpuDbPositionType position_type, 
                               const char serial_number[], Interval &interval,
                               double &datum_offset);
    bool fpuDbTransferCounters(DbTransferType transfer_type,
                               const char serial_number[],
                               FpuCounters &fpu_counters);
    bool fpuDbTransferWaveform(DbTransferType transfer_type,
                               const char serial_number[],
                               t_waveform_steps &waveform);
    bool fpuDbTransferInt64Val(DbTransferType transfer_type,
                               FpuDbIntValType intval_type,
                               const char serial_number[],
                               int64_t &int64_val);
    bool fpuDbTransferWfReversedFlag(DbTransferType transfer_type,
                                     const char serial_number[],
                                     bool &wf_reversed);

    bool fpuDbWriteItem(const char serial_number[], const char subkey[],
                        void *data_ptr, int num_bytes);
    bool fpuDbGetItemDataPtrAndSize(const char serial_number[],
                                    const char subkey[], void **data_ptr_ret,
                                    int &num_bytes_ret);

    bool fpuDbGetSerialNumbers(std::vector<std::string> &serial_numbers_ret);

    ~ProtectionDbTxn();

private:
    static MDB_val fpuDbCreateKeyVal(const char serial_number[], // N.B. Static
                                     const char subkey[]);
    static bool fpuDbGetSerialNumFromKeyVal(const MDB_val &key_val, // N.B. Static
                                            std::string &serial_number_ret);

    MDB_env *env_ptr = nullptr;
    MDB_txn *txn_ptr = nullptr;
};

//==============================================================================

// Forward reference for friend-ing in ProtectionDB below
class ProtectionDBTester;

using ProtectionDbTxnPtr = std::unique_ptr<ProtectionDbTxn>;

class ProtectionDB
{
    // Declare test class as friend so that it can access protected/private
    // member variables and functions
    friend class ProtectionDBTester;

public:
    static std::string getDirFromLinuxEnv(bool mockup);  // N.B. Static

    bool open(const std::string &dir_str);
    ProtectionDbTxnPtr createTransaction();
    bool sync();
    ~ProtectionDB();
    
private:
    void close();

    MDB_env *mdb_env_ptr = nullptr;
};

//==============================================================================

#endif // PROTECTIONDB_H
