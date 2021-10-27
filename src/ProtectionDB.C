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
// NAME ProtectionDB.C
//
// MOONS grid driver database interface layer for reading and writing FPU data
// items. This database uses an LMDB database for its storage, which consists
// of a pair of LMDB-format "environment" files, data.mdb abd lock.mdb, Inside
// this database, a few grid-driver-specific LMDB "sub-databases" are used, for
// example "fpu_db" and others.
//
// During use, the files and the directory that they are in must have the
// appropriate read/write permissions.
//
// ************************** IMPORTANT NOTE: **********************************
// The format of this C++ grid driver LMDB database is NOT COMPATIBLE with the
// older Python-based grid driver LMDB database format:
//   - This C++ version uses a binary format for the FPU data items inside the
//     LMDB database, whereas the older Python version uses a Python-native
//     ASCII-like format
//   - In this C++ version, the sub-databases inside the main database have
//     deliberately been given different names to that of the Python version
//     (e.g. "fpu_db" versus "fpu" respectively for the FPU sub-database), so
//     that any attempted accidental cross-use generates errors - see the
//     comments above fpu_subdb_name and the other sub-database name
//     definitions below
//
// ===============================
// LMDB generic command-line tools
// ===============================
// The third-party LMDB database library provides a number of associated
// command-line utilities which can be used to perform various operations at
// raw LMDB database level:
//   - See http://www.lmdb.tech/doc/tools.html (these tools can be installed
//     as the lmdb-utils package in Linux, or built from source):
//       - mdb_copy
//       - mdb_dump
//       - mdb_load
//       - mdb_stat
//
////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <cstring>
#include <cstdlib>
#include "ProtectionDB.h"

// ProtectionDB sub-database names
// NOTE: This C++ code deliberately uses new sub-database names which are
// different from the old Python version (which instead uses "fpu" and
// "healthlog" without the "_db" suffixes), so that it can detect attempted
// accidental cross-use of the incompatible databases
static const char *fpu_subdb_name = "fpu_db";
static const char *old_fpu_subdb_name = "fpu";  // Old Python version's name
// TODO: The health log database implemented in the original Python database
// implementation isn't fleshed out in this C++ version yet
static const char *healthlog_subdb_name = "healthlog_db";
// TODO: Is the "verification" sub-database needed?
// static const char *verification_subdb_name = "verification_db";

// Database key strings
static const char *snum_used_flag_keystr = "snum_used";
static const char *alpha_position_keystr = "apos";
static const char *beta_position_keystr = "bpos";
static const char *waveform_table_keystr = "wtable";
static const char *waveform_reversed_keystr = "wfreversed";
static const char *alpha_limits_keystr = "alimits";
static const char *beta_limits_keystr = "blimits";
static const char *free_alpha_retries_keystr = "aretries";
static const char *alpha_retries_cw_keystr = "aretries_cw";
static const char *alpha_retries_acw_keystr = "aretries_acw";
static const char *free_beta_retries_keystr = "bretries";
static const char *beta_retries_cw_keystr = "bretries_cw";
static const char *beta_retries_acw_keystr = "bretries_acw";
static const char *counters_keystr = "counters";

// Database Linux environment variable definitions
static const char *fpudb_env_str = "FPU_DATABASE_DIR_NEWFORMAT";
static const char *fpudb_env_mockup_str = "FPU_DATABASE_DIR_NEWFORMAT_MOCKUP";

// Sub-database handles - initialised when ProtectionDB.open() is called
static MDB_dbi fpu_dbi = 0;
static MDB_dbi healthlog_dbi = 0;
// TODO: Is "verification" sub-database database needed?
// static MDB_dbi verificationdb_dbi = 0;

// ProtectionDbTxn counter allowing it to check if attempting to create more
// than one transaction instance at a time (which is not allowed)
int ProtectionDbTxn::num_existing_transaction_instances = 0;


//==============================================================================
FpuDbData::FpuDbData()
{
    // Initialise the items which don't initialise themselves
    snum_used_flag = SNUM_USED_CHECK_VAL;
    wf_reversed = false;
    maxaretries = 0;
    aretries_cw = 0;
    aretries_acw = 0;
    maxbretries = 0;
    bretries_cw = 0;
    bretries_acw = 0;

    datum_offsets[(int)FpuDbIntervalType::AlphaPos] = ALPHA_DATUM_OFFSET;
    datum_offsets[(int)FpuDbIntervalType::AlphaLimits] = ALPHA_DATUM_OFFSET;
    datum_offsets[(int)FpuDbIntervalType::BetaPos] = BETA_DATUM_OFFSET;
    datum_offsets[(int)FpuDbIntervalType::BetaLimits] = BETA_DATUM_OFFSET;
}

//------------------------------------------------------------------------------
bool FpuDbData::operator==(const FpuDbData &other)
{
    return isSameAsOther(other);
}

//------------------------------------------------------------------------------
bool FpuDbData::operator!=(const FpuDbData &other)
{
    return !isSameAsOther(other);
}

//------------------------------------------------------------------------------
bool FpuDbData::isSameAsOther(const FpuDbData &other)
{
    bool waveforms_are_equal = false;
    if (last_waveform.size() == other.last_waveform.size())
    {
        waveforms_are_equal = true;
        for (size_t i = 0; i < last_waveform.size(); i++)
        {
            if ((last_waveform[i].alpha_steps !=
                    other.last_waveform[i].alpha_steps) ||
                (last_waveform[i].beta_steps !=
                    other.last_waveform[i].beta_steps))
            {
                waveforms_are_equal = false;
                break;
            }
        }
    }

    bool offsets_are_equal = true;
    for (int i = 0; i < (int)FpuDbIntervalType::NumTypes; i++)
    {
        if (datum_offsets[i] != other.datum_offsets[i])
        {
            offsets_are_equal = false;
            break;
        }
    }

    return ((snum_used_flag == other.snum_used_flag) &&
            (apos == other.apos) &&
            (bpos == other.bpos) &&
            (wf_reversed == other.wf_reversed) &&
            (alimits == other.alimits) &&
            (blimits == other.blimits) &&
            (maxaretries == other.maxaretries) &&
            (aretries_cw == other.aretries_cw) &&
            (aretries_acw == other.aretries_acw) &&
            (maxbretries == other.maxbretries) &&
            (bretries_cw == other.bretries_cw) &&
            (bretries_acw == other.bretries_acw) &&
            (counters == other.counters) &&
            waveforms_are_equal && 
            offsets_are_equal);
}

//==============================================================================
void ProtectionDB::getLinuxEnvVariableNames(std::string &main_dir_env_name_ret,
                                            std::string &mockup_dir_env_name_ret)
{
    // Provides the names of the FPU database directory environment variables
    // which the grid driver expects. N.B. It's their names, rather than what
    // they are set to, which are returned.

    main_dir_env_name_ret = fpudb_env_str;
    mockup_dir_env_name_ret = fpudb_env_mockup_str;
}

//------------------------------------------------------------------------------
std::string ProtectionDB::getDirFromLinuxEnv(bool use_mockup_db)
{
    // Provides a Linux directory path for the protection database based upon
    // the Linux environment variables FPU_DATABASE_DIR_NEWFORMAT and
    // FPU_DATABASE_DIR_NEWFORMAT_MOCKUP, and the value of use_mockup_db. If
    // unsuccessful then the returned string is empty.
    // The environment variables need to be of the form e.g. "/var/lib/fpudb",
    // and must **NOT** have a final "/" character.

    char *dir_c_str = nullptr;
    std::string dir_str_ret;

    if (!use_mockup_db)
    {
        dir_c_str = getenv(fpudb_env_str);
        if (dir_c_str != nullptr)
        {
            dir_str_ret = dir_c_str;
        }
    }
    else
    {
        dir_c_str = getenv(fpudb_env_mockup_str);
        if (dir_c_str != nullptr)
        {
            dir_str_ret = dir_c_str;
        }
        else
        {
            dir_c_str = getenv(fpudb_env_str);
            if (dir_c_str != nullptr)
            {
                dir_str_ret = dir_c_str;
                dir_str_ret += "_MOCKUP";
            }
        }
    }

    return dir_str_ret;
}

//------------------------------------------------------------------------------
MdbResult ProtectionDB::createEmpty(const std::string &dir_str)
{
    // Creates an empty protection database LMDB "environment" (data.mdb and
    // lock.mdb files) in the specified directory, and creates the empty
    // sub-databases inside it which are required for the grid driver.
    // Notes:
    //   - The directory must already exist and have the appropriate read/write
    //     permissions
    //   - See comments in openOrCreate() for the required dir_str format
    //   - The database is closed again immediately after creation, and open()
    //     must then subsequently be called to open and use it
    //   - Possible outcomes:
    //       - If successful then returns MDB_SUCCESS
    //       - If a database already exists in the specified directory then the
    //         function aborts and returns MDB_DB_ALREADY_EXISTS - this is to
    //         prevent accidental overwriting of an existing database
    //       - If any other problems then returns various other MdbResult/LMDB
    //         return codes
    //   - This function is provided because the standard LMDB command-line tools
    //     (see comments at top of this file) don't provide any database
    //     creation capability

    const std::string data_mdb_file_path = dir_str + "/data.mdb";
    if (access(data_mdb_file_path.c_str(), F_OK) == 0)
    {
        return MDB_DB_ALREADY_EXISTS;
    }

    return openOrCreate(dir_str, OpenOrCreate::Create);
}

//------------------------------------------------------------------------------
MdbResult ProtectionDB::open(const std::string &dir_str)
{
    // Opens an already-existing grid driver protection database environment
    // (data.mdb and lock.mdb files) in the location pointed to by dir_str.
    // Notes:
    //   - See comments in openOrCreate() for the required dir_str format
    //   - If the files do not exist in the specified directory, or they (or
    //     the specified directory) do not have the required read and write
    //     permissions, then this function fails

    // *** IMPORTANT NOTE ***: This C++ grid driver database interface is NOT
    // COMPATIBLE with the older, original Python-based database interface
    // format - see comments at top of this file.
    
    // TODO: Must only call exactly once for a particular LMDB file in this
    // process (see the LMDB documentation) - enforce this somehow?

    //..........................................................................
    // Check that the database environment files exist (data.mdb and
    // lock.mdb) and they they have the correct read/write permissions
    const std::string data_mdb_file_path = dir_str + "/data.mdb";
    const std::string lock_mdb_file_path = dir_str + "/lock.mdb";

    // Check existence
    if ((access(data_mdb_file_path.c_str(), F_OK) != 0) ||
        (access(lock_mdb_file_path.c_str(), F_OK) != 0))
    {
        return ENOENT;
    }

    // Check access permissions
    const int access_permissions = R_OK | W_OK;
    if (access(data_mdb_file_path.c_str(), access_permissions) != 0)
    {
        return errno;
    }
    if (access(lock_mdb_file_path.c_str(), access_permissions) != 0)
    {
        return errno;
    }

    // Open database
    return openOrCreate(dir_str, OpenOrCreate::Open);
}

//------------------------------------------------------------------------------
MdbResult ProtectionDB::openOrCreate(const std::string &dir_str,
                                     OpenOrCreate open_or_create)
{
    // Opens or creates a grid driver protection database, which consists of
    // the data.mdb and lock.mdb "environment" files and the grid-driver-
    // specific sub-databases inside them.
    //
    // dir_str must be of the general form e.g. "/var/lib/fpudb", and must
    // **NOT** have a final "/" character.
    //
    // This function combines the open and create operations for the following
    // reasons:
    //   - The code is almost the same for both operations because the LMDB C
    //     library code supports opening or creation of the "environment" files
    //     and sub-databases from the same functions
    //   - Having both of these operations together in this function ensures
    //     that any created database's settings are compatible with their
    //     subsequent opening settings

    //..........................................................................

    unsigned int dbi_open_flags;
    if (open_or_create == OpenOrCreate::Create)
    {
        dbi_open_flags = MDB_CREATE;
    }
    else
    {
        dbi_open_flags = 0x0;
    }

    size_t dbsize;

    // Needs 64 bit (large file support) for normal database size
    // TODO: In original Python code, 32/64-bit OS check was done - but is
    // this possible from C++? Or can we assume it's always 64-bit?
    // (can make it a requirement?)
    // if (platform.architecture()[0] == "64bit")  // Original Python
    if (1)
    {
        dbsize = 5 * 1024L * 1024L * 1024L;
    }
    else
    {
        dbsize = 5 * 1024L * 1024L;
    }

    //..........................................................................
    // Open/create database environment
    MdbResult mdb_result = mdb_env_create(&mdb_env_ptr);
    if (mdb_result == MDB_SUCCESS)
    {
        mdb_result = mdb_env_set_maxdbs(mdb_env_ptr, 10);
    }
    
    if (mdb_result == MDB_SUCCESS)
    {
        mdb_result = mdb_env_set_mapsize(mdb_env_ptr, dbsize);
    }

    if (mdb_result == MDB_SUCCESS)
    {
        // TODO: The following keeps compatibility with the original
        // Python code defaults, but are this many readers needed?
        mdb_result = mdb_env_set_maxreaders(mdb_env_ptr, 126);
    }

    if (mdb_result == MDB_SUCCESS)
    {
        // Open the "environment" (data.mdb/lock.mdb files) - if they don't
        // exist then they are created
        // N.B. Using default flags for now, so flags value is 0x0
        // N.B. mdb_env_open() documentation says "If this function fails,
        // mdb_env_close() must be called to discard the MDB_env handle" - this
        // is done in the subsequent close() function call
        // TODO: Check the required state of MDB_NOTLS flag
        unsigned int flags = 0x0;
        // ************************* TODO: Check that the following UNIX permissions are OK - I've had
        // ************************* them as 0755 until now, but don't want execute permissions, so is
        // 0666 OK?
        const mdb_mode_t mdb_permissions = 0666;
        mdb_result = mdb_env_open(mdb_env_ptr, dir_str.c_str(), flags,
                                  mdb_permissions);
    }

    //..........................................................................
    // Create sub-database handles - will use for entire time that ProtectionDB
    // is open
    MDB_txn *txn_ptr = nullptr;
    if (mdb_result == MDB_SUCCESS)
    {
        // Create dummy transaction
        mdb_result = mdb_txn_begin(mdb_env_ptr, nullptr, 0x0, &txn_ptr);
    }

    if (mdb_result == MDB_SUCCESS)
    {
        // Check if trying to open an old incompatible Python database, by
        // checking if there is an FPU sub-database present with the old name
        if (open_or_create == OpenOrCreate::Open)
        {
            if (mdb_dbi_open(txn_ptr, old_fpu_subdb_name, 0x0,
                             &fpu_dbi) == MDB_SUCCESS)
            {
                mdb_result = MDB_OLD_INCOMPATIBLE_DB_FORMAT;
            }
        }
    }
    
    if (mdb_result == MDB_SUCCESS)
    {
        // Create FPU sub-database handle - if dbi_open_flags specifies
        // MDB_CREATE then will also create the sub-database if doesn't exist
        mdb_result = mdb_dbi_open(txn_ptr, fpu_subdb_name, dbi_open_flags,
                                  &fpu_dbi);
        if ((open_or_create == OpenOrCreate::Open) &&
            (mdb_result == MDB_NOTFOUND))
        {
            mdb_result = MDB_FPU_SUBDB_MISSING;
        }
    }
    
    if (mdb_result == MDB_SUCCESS)
    {
        // Create health log sub-database handle - if dbi_open_flags specifies
        // MDB_CREATE then will also create the sub-database if doesn't exist
        mdb_result = mdb_dbi_open(txn_ptr, healthlog_subdb_name,
                                  dbi_open_flags, &healthlog_dbi);
        if ((open_or_create == OpenOrCreate::Open) &&
            (mdb_result == MDB_NOTFOUND))
        {
            mdb_result = MDB_HEALTHLOG_SUBDB_MISSING;
        }
    }
    
    if (mdb_result == MDB_SUCCESS)
    {
        // Commit dummy transaction to finish with it
        mdb_result = mdb_txn_commit(txn_ptr);
    }
    
    //..........................................................................
    
    if ((mdb_result != MDB_SUCCESS) ||
        (open_or_create == OpenOrCreate::Create))
    {
        close();
    }
    return mdb_result;
}

//------------------------------------------------------------------------------
ProtectionDbTxnPtr ProtectionDB::createTransaction(MdbResult &mdb_result_ret)
{
    // Creates a protection database transaction.
    // NOTE: Must only have a single transaction instance in existence at a
    // time, so need to ensure that an existing one is destroyed (e.g. by it
    // going out of scope) before creating a new one. See LMDB mdb_txn_begin()
    // web documentation: "A thread may only have a single transaction at a
    // time". This is checked by the ProtectionDbTxn instance being created,
    // and produces MDB_CREATING_MORE_THAN_ONE_TRANSACTION_OBJECT in
    // mdb_result_ret if it occurs. This is a useful check because the LMDB
    // library otherwise indefinitely hangs on a mutex if this is attempted.
    //
    // Return values:
    //   - If successful then the returned ProtectionDbTxnPtr contains a valid
    //     pointer, and mdb_result_ret will contain MDB_SUCCESS
    //   - If not successful then the returned ProtectionDBTxnPtr contains a
    //     null pointer, and mdb_result_ret will contain a database error code

    ProtectionDbTxnPtr ptr_returned;

    mdb_result_ret = MDB_PANIC;

    if (mdb_env_ptr != nullptr)
    {
        // TODO: C++11 doesn't support make_unique - ask if OK to compile
        // as C++14 in final ESO driver
        //ptr_returned = std::make_unique<FpuDbTxn>(*_mdb_env_ptr, created_ok);
        ptr_returned.reset(new ProtectionDbTxn(mdb_env_ptr, mdb_result_ret));
        if (mdb_result_ret != MDB_SUCCESS)
        {
            ptr_returned.reset();
        }
    }
    return std::move(ptr_returned);
}

//------------------------------------------------------------------------------
MdbResult ProtectionDB::sync()
{
    // Flushes the database buffers to disk.

    MdbResult mdb_result = MDB_PANIC;

    if (mdb_env_ptr != nullptr)
    {
        int force = 0;    // TODO: Is this the correct flag setting?
        mdb_result = mdb_env_sync(mdb_env_ptr, force);
    }

    return mdb_result;
}

//------------------------------------------------------------------------------
void ProtectionDB::close()
{
    // Releases all handles, closes ProtectionDB LMDB environment etc.
    if (mdb_env_ptr != nullptr)
    {
        // TODO: mdb_dbi_close() has caveats - see its documentation - but this
        // should be OK here?
        mdb_dbi_close(mdb_env_ptr, fpu_dbi);
        mdb_dbi_close(mdb_env_ptr, healthlog_dbi);
        
        mdb_env_close(mdb_env_ptr);
    
        mdb_env_ptr = nullptr;
        
        // TODO: Call any others? e.g. 
        // mdb_env_sync()?
        // Any others?
    }
}
    
//------------------------------------------------------------------------------
ProtectionDB::~ProtectionDB()
{
    close();
}

//------------------------------------------------------------------------------
std::string ProtectionDB::getResultString(MdbResult mdb_result)
{
    // Provides a nicely-formatted result string for the mdb_result value. This
    // is a thin wrapper for the LMDB C library's mdb_strerror() function, but
    // also includes the extra MDB_XXXX error codes defined in ProtectionDB.h.
    // N.B. Also see the comments above the MdbResult define in ProtectionDB.h.

    static const struct
    {
        MdbResult mdb_result;
        const char *str;
    } extra_result_strings[] =
    {
        {
            MDB_VERIFY_FAILED,
            "MDB_VERIFY_FAILED: Value read back did not equal value written"
        },
        {
             MDB_INCORRECT_SNUM_USED_FLAG_VAL,
            "MDB_INCORRECT_SNUM_USED_FLAG_VAL: Incorrect serial-number-used flag value"
        },
        {
            MDB_DB_ALREADY_EXISTS,
            "MDB_DB_ALREADY_EXISTS: A database already exists in the specified directory"
        },
        {
            MDB_OLD_INCOMPATIBLE_DB_FORMAT,
            "MDB_OLD_INCOMPATIBLE_DB_FORMAT: Attempting to open an old incompatible Python-format database"
        },
        {
            MDB_FPU_SUBDB_MISSING,
            "MDB_FPU_SUBDB_MISSING: The FPU sub-database is missing from the database"
        },
        {
            MDB_HEALTHLOG_SUBDB_MISSING,
            "MDB_HEALTHLOG_SUBDB_MISSING: The health log sub-database is missing from the database"
        },
        {
            MDB_CREATING_MORE_THAN_ONE_TRANSACTION_OBJECT,
            "MDB_CREATING_MORE_THAN_ONE_TRANSACTION_OBJECT: Only one transaction object should exist at a time"
        },
        {
            MDB_INVALID_TRANSACTION_BEING_USED,
            "MDB_INVALID_TRANSACTION_BEING_USED: An invalid transaction object is being used (error during its creation?)"
        }
    };

    std::string result_string;
                                
    if ((mdb_result > MDB_EXTRA_RESULT_CODES_LOWER) &&
        (mdb_result < MDB_EXTRA_RESULT_CODES_UPPER))
    {
        bool foundExtra = false;
        for (size_t i = 0;
             i < (sizeof(extra_result_strings) / sizeof(extra_result_strings[0]));
             i++)
        {
            if (extra_result_strings[i].mdb_result == mdb_result)
            {
                result_string = extra_result_strings[i].str;
                foundExtra = true;
                break;
            }
        }
        if (!foundExtra)
        {
            result_string = "**ERROR**: Database result code was not recognised";
        }
    }
    else
    {
        result_string = mdb_strerror(mdb_result);
    }

    return "(" + std::to_string(mdb_result) + "): " + result_string;
}

//==============================================================================
ProtectionDbTxn::ProtectionDbTxn(MDB_env *protectiondb_mdb_env_ptr,
                                 MdbResult &mdb_result_ret)
{
    // N.B. A maximum of only one transaction object instance must exist at a
    // time - see comments in ProtectionDB::createTransaction()
    if (num_existing_transaction_instances == 0)
    {
        env_ptr = protectiondb_mdb_env_ptr;
        mdb_result_ret = mdb_txn_begin(env_ptr, nullptr, 0x0, &txn_ptr);
        if (mdb_result_ret == MDB_SUCCESS)
        {
            txn_object_is_valid = true;
        }
    }
    else
    {
        mdb_result_ret = MDB_CREATING_MORE_THAN_ONE_TRANSACTION_OBJECT;
    }
    num_existing_transaction_instances++;
}

//------------------------------------------------------------------------------
MdbResult ProtectionDbTxn::fpuDbTransferFpu(DbTransferType transfer_type,
                                            const char serial_number[],
                                            FpuDbData &fpu_db_data)
{
    // Reads or writes all data items for a single FPU specified by the given
    // serial_number.
    // NOTE: For a read transfer, specifying DbTransferType::Read versus
    // DbTransferType::ReadRaw will affect how the interval values (apos/bpos/
    // alimits/blimits) are read - see the comments in fpuDbTransferInterval().

    if (!txn_object_is_valid)
    {
        return MDB_INVALID_TRANSACTION_BEING_USED;
    }

    //..........................................................................
    // Serial-number-used flag
    MdbResult mdb_result = fpuDbTransferInt64Val(transfer_type,
                                                 FpuDbIntValType::SnumUsedFlag,
                                                 serial_number,
                                                 fpu_db_data.snum_used_flag);

    //..........................................................................
    // Intervals and their datum offsets (apos/bpos/alimits/blimits)
    if (mdb_result == MDB_SUCCESS)
    {
        mdb_result = fpuDbTransferInterval(transfer_type,
                                           FpuDbIntervalType::AlphaPos,
                                           serial_number, fpu_db_data.apos,
                fpu_db_data.datum_offsets[(int)FpuDbIntervalType::AlphaPos]);
    }

    if (mdb_result == MDB_SUCCESS)
    {
        mdb_result = fpuDbTransferInterval(transfer_type,
                                           FpuDbIntervalType::BetaPos,
                                           serial_number, fpu_db_data.bpos,
                fpu_db_data.datum_offsets[(int)FpuDbIntervalType::BetaPos]);
    }

    if (mdb_result == MDB_SUCCESS)
    {
        mdb_result = fpuDbTransferInterval(transfer_type,
                                           FpuDbIntervalType::AlphaLimits,
                                           serial_number, fpu_db_data.alimits,
                fpu_db_data.datum_offsets[(int)FpuDbIntervalType::AlphaLimits]);
    }

    if (mdb_result == MDB_SUCCESS)
    {
        mdb_result = fpuDbTransferInterval(transfer_type,
                                           FpuDbIntervalType::BetaLimits,
                                           serial_number, fpu_db_data.blimits,
                fpu_db_data.datum_offsets[(int)FpuDbIntervalType::BetaLimits]);
    }

    //..........................................................................
    // Waveform-reversed flag
    if (mdb_result == MDB_SUCCESS)
    {
        mdb_result = fpuDbTransferWfReversedFlag(transfer_type, serial_number,
                                                 fpu_db_data.wf_reversed);
    }

    //..........................................................................
    // Alpha/beta arm retry counts
    // TODO: Does FreeAlphaRetries / FreeBetaRetries actually correspond to
    // maxrareties/maxbretries in the items below?
    if (mdb_result == MDB_SUCCESS)
    {
        mdb_result = fpuDbTransferInt64Val(transfer_type,
                                           FpuDbIntValType::FreeAlphaRetries,
                                           serial_number,
                                           fpu_db_data.maxaretries);
    }

    if (mdb_result == MDB_SUCCESS)
    {
        mdb_result = fpuDbTransferInt64Val(transfer_type,
                                           FpuDbIntValType::AlphaRetries_CW,
                                           serial_number,
                                           fpu_db_data.aretries_cw);
    }

    if (mdb_result == MDB_SUCCESS)
    {
        mdb_result = fpuDbTransferInt64Val(transfer_type,
                                           FpuDbIntValType::AlphaRetries_ACW,
                                           serial_number,
                                           fpu_db_data.aretries_acw);
    }

    if (mdb_result == MDB_SUCCESS)
    {
        mdb_result = fpuDbTransferInt64Val(transfer_type,
                                           FpuDbIntValType::FreeBetaRetries,
                                           serial_number,
                                           fpu_db_data.maxbretries);
    }

    if (mdb_result == MDB_SUCCESS)
    {
        mdb_result = fpuDbTransferInt64Val(transfer_type,
                                           FpuDbIntValType::BetaRetries_CW,
                                           serial_number,
                                           fpu_db_data.bretries_cw);
    }

    if (mdb_result == MDB_SUCCESS)
    {
        mdb_result = fpuDbTransferInt64Val(transfer_type,
                                           FpuDbIntValType::BetaRetries_ACW,
                                           serial_number,
                                           fpu_db_data.bretries_acw);
    }

    //..........................................................................
    // Counters
    if (mdb_result == MDB_SUCCESS)
    {
        mdb_result = fpuDbTransferCounters(transfer_type, serial_number,
                                           fpu_db_data.counters);
    }

    //..........................................................................
    // Last waveform
    if (mdb_result == MDB_SUCCESS)
    {
        mdb_result = fpuDbTransferWaveform(transfer_type, serial_number,
                                           fpu_db_data.last_waveform);
    }

    //..........................................................................

    return mdb_result;
}

//------------------------------------------------------------------------------
MdbResult ProtectionDbTxn::fpuDbTransferInterval(DbTransferType transfer_type,
                                                 FpuDbIntervalType interval_type,
                                                 const char serial_number[],
                                                 Interval &interval,
                                                 double &datum_offset)
{
    // Reads or writes an interval + datum offset combination of type
    // FpuDbIntervalType for an FPU.
    //
    // NOTE: Read transfer types:
    //   - DbTransferType::Read: Adjusts the returned interval by subtracting
    //     the datum offset from the raw interval in the database
    //   - DbTransferType::ReadRaw: Just gets the raw interval + offset values
    //     from the FPU database
    //
    // In theory, it is cleaner to only store the relative values. But we want
    // the database content to be easy to interpret and have a uniform angle
    // interpretation, so it is better to always store interval values along
    // with the offset they refer to. So we store the datum offsets along with
    // each position interval (this allows to reconfigure the zero point later).

    if (!txn_object_is_valid)
    {
        return MDB_INVALID_TRANSACTION_BEING_USED;
    }

    MdbResult mdb_result = MDB_PANIC;

    static const struct
    {
        FpuDbIntervalType type;
        const char *subkey;
    } interval_subkeys[(int)FpuDbIntervalType::NumTypes] = 
    {
        { FpuDbIntervalType::AlphaLimits, alpha_limits_keystr   },
        { FpuDbIntervalType::AlphaPos,    alpha_position_keystr },
        { FpuDbIntervalType::BetaLimits,  beta_limits_keystr    },
        { FpuDbIntervalType::BetaPos,     beta_position_keystr  }
    };

    const char *subkey = nullptr;
    for (int i = 0; i < (int)FpuDbIntervalType::NumTypes; i++)
    {
        if (interval_subkeys[i].type == interval_type)
        {
            subkey = interval_subkeys[i].subkey;
            break;
        }
    }

    if (subkey != nullptr)
    {
        double doubles_array[3];
        if (transfer_type == DbTransferType::Write)
        {
            // Write the position item
            interval.getLowerUpper(doubles_array[0], doubles_array[1]);
            doubles_array[2] = datum_offset;
            mdb_result = fpuDbWriteItem(serial_number, subkey, doubles_array,
                                        sizeof(doubles_array));
        }
        else
        {
            // Read the position item
            void *item_data_ptr = nullptr;
            int num_item_data_bytes = 0;
            mdb_result = fpuDbGetItemDataPtrAndSize(serial_number, subkey,
                                                    &item_data_ptr,
                                                    num_item_data_bytes);
            if (mdb_result == MDB_SUCCESS)
            {
                if (num_item_data_bytes == sizeof(doubles_array))
                {
                    memcpy(doubles_array, item_data_ptr, sizeof(doubles_array));

                    datum_offset = doubles_array[2];
                    if (transfer_type == DbTransferType::Read)
                    {
                        // DbTransferType::Read
                        // For this normal interval read (rather than a raw
                        // read), subtract the datum offset value - this
                        // transforms the absolute position and offset into a
                        // relative position. This allows storing of the
                        // intervals independently of their offset, allowing
                        // configurable offsets for the alpha datum position.
                        // BW NOTE: This is equivalent to the original Python
                        // version's ProtectionDB.getField() function ->
                        // interval-specific adjustment code.
                        interval = Interval(doubles_array[0], doubles_array[1]) -
                                   datum_offset;
                    }
                    else
                    {
                        // DbTransferType::ReadRaw
                        interval = Interval(doubles_array[0], doubles_array[1]);
                    }
                    mdb_result = MDB_SUCCESS;
                }
                else
                {
                    mdb_result = MDB_BAD_VALSIZE;
                }
            }
        }
    }

    return mdb_result;
}

//------------------------------------------------------------------------------
MdbResult ProtectionDbTxn::fpuDbTransferCounters(DbTransferType transfer_type,
                                                 const char serial_number[],
                                                 FpuCounters &fpu_counters)
{
    // Reads or writes a set of FPU counters.

    if (!txn_object_is_valid)
    {
        return MDB_INVALID_TRANSACTION_BEING_USED;
    }

    MdbResult mdb_result = MDB_PANIC;

    if (transfer_type == DbTransferType::Write)
    {
        mdb_result = fpuDbWriteItem(serial_number, counters_keystr,
                                    fpu_counters.getRawBytesPtr(),
                                    fpu_counters.getNumRawBytes());
    }
    else
    {
        void *item_data_ptr = nullptr;
        int num_item_bytes = 0;
        mdb_result = fpuDbGetItemDataPtrAndSize(serial_number, counters_keystr,
                                                &item_data_ptr, num_item_bytes);
        if (mdb_result == MDB_SUCCESS)
        {
            if (num_item_bytes == fpu_counters.getNumRawBytes())
            {
                fpu_counters.populateFromRawBytes(item_data_ptr);
                mdb_result = MDB_SUCCESS;
            }
            else
            {
                mdb_result = MDB_BAD_VALSIZE;
            }
        }
    }

    return mdb_result;
}

//------------------------------------------------------------------------------
MdbResult ProtectionDbTxn::fpuDbTransferWaveform(DbTransferType transfer_type,
                                                 const char serial_number[],
                                                 t_waveform_steps &waveform)
{
    // Reads or writes a forward or reversed waveform.

    if (!txn_object_is_valid)
    {
        return MDB_INVALID_TRANSACTION_BEING_USED;
    }

    MdbResult mdb_result = MDB_PANIC;

    // NOTE: StepsIntType matches the t_step_pair.alpha_steps and
    // t_step_pair.beta_steps types
    using StepsIntType = int16_t;

    if (transfer_type == DbTransferType::Write)
    {
        // Pack waveform values into a byte buffer to ensure that the data
        // packing will always be consistent
        std::vector<uint8_t> bytesBuf(waveform.size() * 
                                      sizeof(StepsIntType) * 2);
        StepsIntType *intBufPtr = (StepsIntType *)bytesBuf.data();
        for (size_t i = 0; i < waveform.size(); i++)
        {
            *(intBufPtr + (i * 2)) = waveform[i].alpha_steps;
            *(intBufPtr + ((i * 2) + 1)) = waveform[i].beta_steps;
        }

        mdb_result = fpuDbWriteItem(serial_number, waveform_table_keystr,
                                    bytesBuf.data(), bytesBuf.size());
    }
    else
    {
        void *item_data_ptr = nullptr;
        int num_item_bytes = 0;
        mdb_result = fpuDbGetItemDataPtrAndSize(serial_number,
                                                waveform_table_keystr,
                                                &item_data_ptr,
                                                num_item_bytes);
        if (mdb_result == MDB_SUCCESS)
        {
            // Check that number of bytes is a multiple of the combined
            // 2 x StepsIntType sizes of t_step_pair
            if ((num_item_bytes % (sizeof(StepsIntType) * 2)) == 0)
            {
                // Resize waveform_entry and unpack the values into it 
                waveform.resize(num_item_bytes / (sizeof(StepsIntType) * 2));
                StepsIntType *intBufPtr = (StepsIntType *)item_data_ptr;
                for (size_t i = 0; i < waveform.size(); i++)
                {
                    waveform[i].alpha_steps = *(intBufPtr + (i * 2));
                    waveform[i].beta_steps = *(intBufPtr + ((i * 2) + 1));
                }
                mdb_result = MDB_SUCCESS;
            }
            else
            {
                waveform.resize(0);
                mdb_result = MDB_BAD_VALSIZE;
            }
        }
    }

    return mdb_result;
}

//------------------------------------------------------------------------------
MdbResult ProtectionDbTxn::fpuDbTransferInt64Val(DbTransferType transfer_type,
                                                 FpuDbIntValType intval_type,
                                                 const char serial_number[],
                                                 int64_t &int64_val)
{
    // Reads or writes an int64_t value.

    if (!txn_object_is_valid)
    {
        return MDB_INVALID_TRANSACTION_BEING_USED;
    }

    MdbResult mdb_result = MDB_PANIC;

    static const struct
    {
        FpuDbIntValType type;
        const char *subkey;
    } intval_type_subkeys[(int)FpuDbIntValType::NumTypes] =
    {
        { FpuDbIntValType::SnumUsedFlag,     snum_used_flag_keystr     },
        { FpuDbIntValType::FreeAlphaRetries, free_alpha_retries_keystr },
        { FpuDbIntValType::AlphaRetries_CW,  alpha_retries_cw_keystr   },
        { FpuDbIntValType::AlphaRetries_ACW, alpha_retries_acw_keystr  }, 
        { FpuDbIntValType::FreeBetaRetries,  free_beta_retries_keystr  },
        { FpuDbIntValType::BetaRetries_CW,   beta_retries_cw_keystr    },
        { FpuDbIntValType::BetaRetries_ACW,  beta_retries_acw_keystr   }
    };

    const char *subkey = nullptr;
    for (int i = 0; i < (int)FpuDbIntValType::NumTypes; i++)
    {
        if (intval_type_subkeys[i].type == intval_type)
        {
            subkey = intval_type_subkeys[i].subkey;
            break;
        }
    }

    if (subkey != nullptr)
    {
        if (transfer_type == DbTransferType::Write)
        {
            mdb_result = fpuDbWriteItem(serial_number, subkey, &int64_val,
                                        sizeof(int64_t));
        }
        else
        {
            void *item_data_ptr = nullptr;
            int num_item_bytes = 0;
            mdb_result = fpuDbGetItemDataPtrAndSize(serial_number, subkey,
                                                    &item_data_ptr,
                                                    num_item_bytes);
            if (mdb_result == MDB_SUCCESS)    
            {
                if (num_item_bytes == sizeof(int64_t))
                {
                    memcpy(&int64_val, item_data_ptr, sizeof(int64_t));
                    mdb_result = MDB_SUCCESS;
                }
                else
                {
                    mdb_result = MDB_BAD_VALSIZE;
                }
            }
        }
    }

    return mdb_result;
}

//------------------------------------------------------------------------------
MdbResult ProtectionDbTxn::fpuDbTransferWfReversedFlag(DbTransferType transfer_type,
                                                       const char serial_number[],
                                                       bool &wf_reversed)
{
    // Reads or writes the waveform-reversed bool flag.

    if (!txn_object_is_valid)
    {
        return MDB_INVALID_TRANSACTION_BEING_USED;
    }

    MdbResult mdb_result = MDB_PANIC;

    if (transfer_type == DbTransferType::Write)
    {
        uint8_t wf_reversed_byte_val;
        if (wf_reversed)
        {
            wf_reversed_byte_val = 1;
        }
        else
        {
            wf_reversed_byte_val = 0;
        }
        mdb_result = fpuDbWriteItem(serial_number, waveform_reversed_keystr,
                                    &wf_reversed_byte_val, sizeof(uint8_t));
    }
    else
    {
        void *item_data_ptr = nullptr;
        int num_item_bytes = 0;
        mdb_result = fpuDbGetItemDataPtrAndSize(serial_number,
                                                waveform_reversed_keystr,
                                                &item_data_ptr, num_item_bytes);
        if (mdb_result == MDB_SUCCESS)    
        {
            if (num_item_bytes == sizeof(uint8_t))
            {
                uint8_t wf_reversed_byte_val = *((uint8_t *)item_data_ptr);
                if (wf_reversed_byte_val != 0)
                {
                    wf_reversed = true;
                }
                else
                {
                    wf_reversed = false;
                }
                mdb_result = MDB_SUCCESS;
            }
            else
            {
                mdb_result = MDB_BAD_VALSIZE;
            }
        }
    }

    return mdb_result;
}

//------------------------------------------------------------------------------
MdbResult ProtectionDbTxn::fpuDbWriteItem(const char serial_number[],
                                          const char subkey[], void *data_ptr,
                                          int num_bytes)
{
    // Writes the specified item. If an item with the same serial_number/subkey
    // combination already exists then it is overwritten.
    
    if (!txn_object_is_valid)
    {
        return MDB_INVALID_TRANSACTION_BEING_USED;
    }

    MDB_val key_val = fpuDbCreateKeyVal(serial_number, subkey);
    MDB_val data_val = { (size_t)num_bytes, data_ptr };
    return mdb_put(txn_ptr, fpu_dbi, &key_val, &data_val, 0x0);
}

//------------------------------------------------------------------------------
MdbResult ProtectionDbTxn::fpuDbGetItemDataPtrAndSize(const char serial_number[],
                                                      const char subkey[],
                                                      void **data_ptr_ret,
                                                      int &num_bytes_ret)
{
    // For the specified item, gets a pointer to its data and gets its size -
    // this is possible because this is an in-memory database.

    if (!txn_object_is_valid)
    {
        return MDB_INVALID_TRANSACTION_BEING_USED;
    }

    MdbResult mdb_result = MDB_PANIC;

    MDB_val key_val = fpuDbCreateKeyVal(serial_number, subkey);
    MDB_val data_val = { 0, nullptr };
    mdb_result = mdb_get(txn_ptr, fpu_dbi, &key_val, &data_val);
    if (mdb_result == MDB_SUCCESS)
    {
        *data_ptr_ret = data_val.mv_data;
        num_bytes_ret = data_val.mv_size;
    }
    else
    {
        *data_ptr_ret = nullptr;
        num_bytes_ret = 0;
    }

    return mdb_result;
}

//------------------------------------------------------------------------------
MdbResult ProtectionDbTxn::fpuDbGetAllSerialNumbers(
                                std::vector<std::string> &serial_numbers_ret)
{
    if (!txn_object_is_valid)
    {
        return MDB_INVALID_TRANSACTION_BEING_USED;
    }

    MDB_cursor *cursor_ptr;
    MdbResult mdb_result = MDB_PANIC;

    serial_numbers_ret.clear();

    // TODO: Want to get all available serial numbers, even if some of the 
    // retrievals fail? (i.e. don't stop on first failure?)

    // TODO: Consider whether this function should only register the serial
    // numbers of the snum_used_flag items (because their purpose is to
    // indicate whether a serial number is in use), or whether we want to see
    // ALL serial numbers for all items (as this function currently does)

    mdb_result = mdb_cursor_open(txn_ptr, fpu_dbi, &cursor_ptr);
    if (mdb_result == MDB_SUCCESS)
    {
        MDB_val key_val;
        MDB_val data_val;
        std::string serial_number;
        std::string prev_serial_number;
        MDB_cursor_op cursor_op = MDB_FIRST;    // Start from first item
        while (1)
        {
            mdb_result = mdb_cursor_get(cursor_ptr, &key_val, &data_val, 
                                        cursor_op);
            cursor_op = MDB_NEXT;
            if (mdb_result == MDB_SUCCESS)
            {
                fpuDbGetSerialNumFromKeyVal(key_val, serial_number);
                if (serial_number.size() != 0)
                {
                    // Each FPU serial number has a number of items on it in
                    // the database (each with a different sub-key suffix) -
                    // only record each serial number once. N.B. This works
                    // because the keys are always stored in order in the
                    // database
                    if (serial_number != prev_serial_number)
                    {
                        serial_numbers_ret.push_back(serial_number);
                        prev_serial_number = serial_number;
                    }
                }
            }
            else
            {
                mdb_cursor_close(cursor_ptr);
                if (mdb_result == MDB_NOTFOUND)
                {
                    // Normal end-of-database reached
                    mdb_result = MDB_SUCCESS;
                }
                else
                {
                    // Unexpected error result
                }
                break;
            }
        }
    }

    return mdb_result;
}

//------------------------------------------------------------------------------
MDB_val ProtectionDbTxn::fpuDbCreateKeyVal(const char serial_number[],
                                           const char subkey[])
{
    // N.B. Need static persistent key_str below because its raw c_str is
    // pointed to by the returned MDB_val.
    // TODO: This is NOT thread-safe (because single static instance) - will
    // this be a problem?
    static std::string key_str;

    // Create ASCII key of form <serial_number><separator><subkey>
    // IMPORTANT: serial_number and subkey must not contain the
    // fpudb_keystr_separator_char character. *** TODO ***: Check for this here?
    key_str = std::string(serial_number) + fpudb_keystr_separator_char + subkey;
    return { key_str.size(), const_cast<void *>((const void *)key_str.c_str()) };
}

//------------------------------------------------------------------------------
bool ProtectionDbTxn::fpuDbGetSerialNumFromKeyVal(const MDB_val &key_val,
                                                  std::string &serial_number_ret)
{
    // Extracts the serial number string from the beginning of key_val (up
    // until the fpudb_keystr_separator_char character). If successful then
    // returns true, otherwise if any problems then still tries to populate
    // serial_number_ret but returns false.
    // N.B. The maximum serial number length is given by 
    // ethercanif::DIGITS_SERIAL_NUMBER, but a bigger buffer is used so that
    // can catch any errant serial numbers of greater length as well
    static const int snum_buf_len = 20;
    char serial_num_buf[snum_buf_len];

    memset(serial_num_buf, '\0', snum_buf_len); // Ensure a null-terminator
    serial_number_ret.clear();

    bool separator_char_found = false;
    const char *source_buf = (char *)key_val.mv_data;
    for (size_t i = 0; ((i < (snum_buf_len - 1)) && (i < key_val.mv_size)); i++)
    {
        if (source_buf[i] == fpudb_keystr_separator_char)
        {
            separator_char_found = true;
            break;
        }
        else if (source_buf[i] == '\0')
        {
            break;
        }
        else
        {
            serial_num_buf[i] = source_buf[i];
        }
    }

    serial_number_ret = serial_num_buf;
    
    if ((!separator_char_found) || (serial_number_ret.size() == 0) ||
        (serial_number_ret.size() > ethercanif::DIGITS_SERIAL_NUMBER))
    {
        return false;
    }
    return true;
}

//------------------------------------------------------------------------------
ProtectionDbTxn::~ProtectionDbTxn()
{
    if (txn_object_is_valid)
    {
        mdb_txn_commit(txn_ptr);
    }

    if (num_existing_transaction_instances > 0)
    {
        num_existing_transaction_instances--;
    }
   
    // TODO: Where to call mdb_env_sync() to flush to disk? Or is this done
    // automatically due to the LMDB config macros / settings?
    // NOTE: In FpuGridDriver.py -> GridDriver, is done in a number of places -
    // search for "self.env.sync()"

}

//==============================================================================

