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
#include "FPUCounters.h"

using namespace mpifps;
using namespace mpifps::ethercanif;

//==============================================================================

// FpuDbData: FPU data which is stored in the protection database.
struct FpuDbData
{
    FpuDbData();
    bool operator==(const FpuDbData &other);
    bool operator!=(const FpuDbData &other);
    
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

    ~ProtectionDbTxn();

private:
    MDB_val fpuDbCreateKeyVal(const char serial_number[],
                              const char subkey[]);

    MDB_env *env_ptr = nullptr;
    MDB_txn *txn_ptr = nullptr;
};

//==============================================================================

// Forward reference for friend-ing in ProtectionDB below
class ProtectionDBTester;

class ProtectionDB
{
    // Declare test class as friend so that it can access protected/private
    // member variables and functions
    friend class ProtectionDBTester;

public:
    static std::string getDirFromLinuxEnv(bool mockup);  // N.B. Static

    bool open(const std::string &dir_str);
    std::unique_ptr<ProtectionDbTxn> createTransaction();
    bool sync();
    ~ProtectionDB();
    
private:
    void close();

    MDB_env *mdb_env_ptr = nullptr;
};

//==============================================================================

#endif // PROTECTIONDB_H
