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

//..............................................................................

enum class DbTransferType
{
    // For reading, there are two options, which specify the way in which the
    // apos/alimits/bpos/blimits intervals are read:
    //   - Read: The returned intervals have their corresponding datum offset
    //     values in the database subtracted from them
    //   - ReadRaw: Doesn't subtract the datum offsets from the intervals -
    //     just reads the raw database intervals
    Read,
    ReadRaw,
    
    Write
};

enum class FpuDbIntervalType
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

// MdbResult: The result/return type for most of the database-related functions
// in ProtectionDB and ProjectionDBTxn. Notes:
//   - Defined in relation to the low-level LMDB C library int return value
//     type
//   - The original LMDB return types represent two different code types/ranges:
//       - LMDB-specific MDB_XXXX return codes, which are zero or negative
//         values - these are defined in lmdb.h - also see
//         http://www.lmdb.tech/doc/group__errors.html
//       - A small number of GNU/Linux "errno" return codes such as EACCES,
//         ENOMEM etc - see them in the LMDB C library's mdb_strerror() 
//         function, see https://man7.org/linux/man-pages/man3/errno.3.html etc
//   - MDB_SUCCESS (0) indicates a successful operation
//   - Some further MDB_XXXX return codes have also been added below, to
//     indicate some protection-database-level errors (with their values being
//     in a different range from the standard LMDB ones)
//   - Use ProtectionDB::getResultString() to get a nicely-formatted result
//     string from the result value
//   - (N.B. In the LMDB C library's mdb_strerror() function, says of the
//     return codes: "A major mess, we should have used LMDB-specific error
//     codes for everything")
enum
{
    // Boundary specifically outside normal LMDB range - do not use or change
    MDB_EXTRA_RESULT_CODES_LOWER = -25000, 

    // Further result codes to use
    // ***NOTE***: These also need corresponding entries in ProtectionDB::
    // getResultString()
    MDB_VERIFY_FAILED,
    MDB_INCORRECT_SNUM_USED_FLAG_VAL,
    MDB_DB_ALREADY_EXISTS,
    MDB_OLD_INCOMPATIBLE_DB_FORMAT,

    // Boundary - do not use or change
    MDB_EXTRA_RESULT_CODES_UPPER
};
typedef int MdbResult;


//==============================================================================

// FpuDbData: FPU data which is stored in the protection database.
struct FpuDbData
{
    FpuDbData();
    bool operator==(const FpuDbData &other);
    bool operator!=(const FpuDbData &other);
    
     // snum_used_flag: In the database, this field's existence for a serial
     // number is used to indicate that the serial number is in use for an FPU
    int64_t snum_used_flag;

    // Intervals note: When one or all of these intervals is read from the
    // FPU database (using fpuDbTransferInterval() or fpuDbTransferFpu()),
    // the 2 different DbTransferType read options specify how these intervals
    // will be populated - see the comments in the
    // ProtectionDbTxn::fpuDbTransferInterval() function.
    Interval apos;
    Interval bpos;
    Interval alimits;
    Interval blimits;

    bool wf_reversed;
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

    // Datum offsets
    // These will reflect the raw datum offset values corresponding to the
    // apos/bpos/alimits/blimits intervals for the FPU, whenever the full set
    // of FPU data is read from the database using fpuDbTransferFpu().
    // However, note that generally the apos/alimits/ bpos/blimits interval
    // values above will have these offsets subtracted automatically when the
    // FPU data is read from the database with DbTransferType::Read specifid
    // (unless DbTransferType::ReadRaw is used instead).
    double datum_offsets[(int)FpuDbIntervalType::NumTypes];
    
private:
    bool isSameAsOther(const FpuDbData &other);
};

//==============================================================================

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
    ProtectionDbTxn(MDB_env *protectiondb_mdb_env_ptr,
                    MdbResult &mdb_result_ret);

    MdbResult fpuDbTransferFpu(DbTransferType transfer_type,
                               const char serial_number[],
                               FpuDbData &fpu_db_data);

    MdbResult fpuDbTransferInterval(DbTransferType transfer_type,
                                    FpuDbIntervalType interval_type,
                                    const char serial_number[],
                                    Interval &interval,
                                    double &datum_offset);
    MdbResult fpuDbTransferCounters(DbTransferType transfer_type,
                                    const char serial_number[],
                                    FpuCounters &fpu_counters);
    MdbResult fpuDbTransferWaveform(DbTransferType transfer_type,
                                    const char serial_number[],
                                    t_waveform_steps &waveform);
    MdbResult fpuDbTransferInt64Val(DbTransferType transfer_type,
                                    FpuDbIntValType intval_type,
                                    const char serial_number[],
                                    int64_t &int64_val);
    MdbResult fpuDbTransferWfReversedFlag(DbTransferType transfer_type,
                                          const char serial_number[],
                                          bool &wf_reversed);

    MdbResult fpuDbGetAllSerialNumbers(std::vector<std::string> &serial_numbers_ret);

    ~ProtectionDbTxn();

private:
    MdbResult fpuDbWriteItem(const char serial_number[], const char subkey[],
                             void *data_ptr, int num_bytes);
    MdbResult fpuDbGetItemDataPtrAndSize(const char serial_number[],
                                         const char subkey[],
                                         void **data_ptr_ret,
                                         int &num_bytes_ret);
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

    MdbResult createEmpty(const std::string &dir_str);
    MdbResult open(const std::string &dir_str);
    ProtectionDbTxnPtr createTransaction(MdbResult &mdb_result_ret);
    MdbResult sync();
    ~ProtectionDB();

    static std::string getResultString(MdbResult mdb_result); // N.B. Static

private:
    enum class OpenOrCreate
    {
        Open,
        Create
    };

    MdbResult openOrCreate(const std::string &dir_str,
                           OpenOrCreate open_or_create);
    void close();

    MDB_env *mdb_env_ptr = nullptr;
};

//==============================================================================

#endif // PROTECTIONDB_H
