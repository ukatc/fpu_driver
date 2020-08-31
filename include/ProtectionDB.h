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


// TODO: Move elsewhere, and rename to e.g. Waveform?
using Wentry = std::vector<t_step_pair>;

// -----------------------------------------------------------------------------

std::string protectionDB_GetDirFromLinuxEnv(bool mockup);

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

enum class DbWaveformType
{
    Forward,
    Reversed
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
    bool fpuDbTransferCounters(DbTransferType transfer_type,
                               const char serial_number[],
                               FpuCounters &fpu_counters);
    bool fpuDbTransferWaveform(DbTransferType transfer_type,
                               DbWaveformType waveform_type,
                               const char serial_number[],
                               Wentry &waveform_entry);
    // TODO: Implement the following functions or similar (adapted from the 
    // Python code)
#if 0
    bool fpuDbPutBetaRetryCount(const char serial_number[], bool clockwise, int count);
    bool fpuDbPutAlphaRetryCount(const char serial_number[], bool clockwise, int count);
#endif // 0

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
