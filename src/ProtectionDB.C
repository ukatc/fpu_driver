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
// NAME ProtectionDB.C
//
// TODO: Put description here
//
////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <cstring>
#include <cstdlib>
#include "ProtectionDB.h"

// ProtectionDB sub-database names
static const char *fpu_subdb_name = "fpu";
static const char *healthlog_subdb_name = "healthlog";
// TODO: Is "verification" sub-database needed?
// static const char *verification_subdb_name = "verification";

// TODO: Disabled for now to avoid build warnings, because not yet used
#if 0
// Database key strings
static const char *alpha_positions_keystr = "apos";
static const char *beta_positions_keystr = "bpos";
static const char *waveform_table_keystr = "wtab";
static const char *waveform_reversed_keystr = "wf_reversed";
static const char *alpha_limits_keystr = "alimits";
static const char *beta_limits_keystr = "blimits";
static const char *free_beta_retries_keystr = "bretries";
static const char *beta_retry_count_cw_keystr = "beta_retry_count_cw";
static const char *beta_retry_count_acw_keystr = "beta_retry_count_acw"; 
static const char *free_alpha_retries_keystr = "aretries";
static const char *alpha_retry_count_cw_keystr = "alpha_retry_count_cw";
static const char *alpha_retry_count_acw_keystr = "alpha_retry_count_acw";
#endif // 0
static const char *counters_keystr = "counters";
#if 0
static const char *serialnumber_used_keystr = "serialnumber_used";
#endif // 0

// Character to separate the key/subkey parts of the overall key strings
static const char *keystr_separator_char = "#";

// Sub-database handles - initialised when ProtectionDB.open() is called
static MDB_dbi fpu_dbi = 0;
static MDB_dbi healthlog_dbi = 0;
// TODO: Is "verification" sub-database database needed?
// static MDB_dbi verificationdb_dbi = 0;


//==============================================================================
std::string protectionDB_GetDirFromLinuxEnv(bool mockup)
{
    // Provides Linux directory for the protection database based upon the
    // Linux environment variables FPU_DATABASE_MOCKUP and FPU_DATABASE, and
    // value of mockup. If unsuccessful then the returned string is empty.
    // The environment variables need to be of the form e.g. "/var/lib/fpudb",
    // and must **NOT** have a final "/" character

    char *dir_c_str = nullptr;
    std::string dir_str_ret;
    
    if (mockup)
    {
        dir_c_str = getenv("FPU_DATABASE_MOCKUP");
        if (dir_c_str == nullptr)
        {
            dir_c_str = getenv("FPU_DATABASE");
            if (dir_c_str != nullptr)
            {
                dir_str_ret = dir_c_str;
                dir_str_ret += "_mockup";
            }
        }
        else
        {
            dir_str_ret = dir_c_str;
        }
    }
    else
    {
        dir_c_str = getenv("FPU_DATABASE");
        if (dir_c_str != nullptr)
        {
            dir_str_ret = dir_c_str;
        }
    }
    
    return dir_str_ret;
}

//==============================================================================
bool ProtectionDB::open(const std::string &dir_str)
{
    // Opens a protection database (data.mdb and locks.mdb files) in the
    // location pointed to by dir_str. Notes:
    //   - If the directory doesn't exist then this function fails
    //   - If the directory exists but the files aren't present in it
    //     then it creates them, i.e. creates a new database
    //   - File and/or directory permissions must have been set up correctly
    //     or this function may fail
    
    // TODO: Must only call exactly once for a particular LMDB file in this
    // process (see the LMDB documentation) - enforce this somehow?
    
    //..........................................................................
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
    // Initialise and open database environment
    // TODO: In the original Python code, the following are defaulted - see
    // https://lmdb.readthedocs.io/en/release/ -> Environment class - 
    // max_spare_txns=1
    // create=True      <== **IMPORTANT** - create directory if doesn't exist?

    int mdb_result = mdb_env_create(&mdb_env_ptr);
    if (mdb_result == 0)
    {
        mdb_result = mdb_env_set_maxdbs(mdb_env_ptr, 10);
    }
    
    if (mdb_result == 0)
    {
        mdb_result = mdb_env_set_mapsize(mdb_env_ptr, dbsize);
    }

    if (mdb_result == 0)
    {
        // TODO: The following keeps compatibility with the original
        // Python code defaults, but are this many readers needed?
        mdb_result = mdb_env_set_maxreaders(mdb_env_ptr, 126);
    }

    if (mdb_result == 0)
    {
        // N.B. Using default flags for now, so flags value is 0x0
        // N.B. Creates new data.mdb abd lock.mdb files if they aren't present
        // TODO: Check the required state of MDB_NOTLS flag
        // TODO: Check that following permissions are OK
        unsigned int flags = 0x0;
        mdb_result = mdb_env_open(mdb_env_ptr, dir_str.c_str(), flags, 0755);
    }

    //..........................................................................
    // Create sub-database handles - will use for entire time that ProtectionDB
    // is open
    // TODO: Check if these operations create the sub-databases if they don't 
    // exist yet, and put comment to this effect here
    MDB_txn *txn_ptr = nullptr;
    if (mdb_result == 0)
    {
        // Create dummy transaction
        mdb_result = mdb_txn_begin(mdb_env_ptr, nullptr, 0x0, &txn_ptr);
    }
    
    if (mdb_result == 0)
    {
        // Create FPU sub-database handle
        mdb_result = mdb_dbi_open(txn_ptr, fpu_subdb_name, MDB_CREATE,
                                  &fpu_dbi);
    }
    
    if (mdb_result == 0)
    {
        // Create health log sub-database handle
        mdb_result = mdb_dbi_open(txn_ptr, healthlog_subdb_name, MDB_CREATE,
                                  &healthlog_dbi);
    }
    
    if (mdb_result == 0)
    {
        // Commit dummy transaction to finish with it
        mdb_result = mdb_txn_commit(txn_ptr);
    }
    
    //..........................................................................
    
    if (mdb_result != 0)
    {
        // Failure - close the database
        close();
        return false;
    }
    return true;
}

//------------------------------------------------------------------------------
std::unique_ptr<ProtectionDbTxn> ProtectionDB::createTransaction()
{
    std::unique_ptr<ProtectionDbTxn> ptr_returned;

    if (mdb_env_ptr != nullptr)
    {
        bool created_ok = false;
        // TODO: C++11 doesn't support make_unique - ask if OK to compile
        // as C++14 in final ESO driver
        //ptr_returned = std::make_unique<FpuDbTxn>(*_mdb_env_ptr, created_ok);
        ptr_returned.reset(new ProtectionDbTxn(mdb_env_ptr, created_ok));
        if (!created_ok)
        {
            ptr_returned.reset();
        }
    }
    return std::move(ptr_returned);
}

//------------------------------------------------------------------------------
void ProtectionDB::close()
{
    // Releases all handles, closes ProtectionDB LMDB environment etc
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

//==============================================================================
ProtectionDbTxn::ProtectionDbTxn(MDB_env *protectiondb_mdb_env_ptr,
                                 bool &created_ok_ret)
{
    env_ptr = protectiondb_mdb_env_ptr;

    created_ok_ret = false;
    if (mdb_txn_begin(env_ptr, nullptr, 0x0, &txn_ptr) == 0)
    {
        created_ok_ret = true;
    }
}

//------------------------------------------------------------------------------
bool ProtectionDbTxn::fpuDbPutCounters(const char serial_number[],
                                       const FpuCounters &fpu_counters)
{
    MDB_val data_val;
    
    data_val.mv_data = fpu_counters.getRawData(data_val.mv_size); 
   
    if (fpuDbPutItem(serial_number, counters_keystr, data_val) == 0)
    {
        return true;
    }
    return false;
}

//------------------------------------------------------------------------------
bool ProtectionDbTxn::fpuDbWriteRawItem(const char serial_number[],
                                        const char subkey[],
                                        void *data_ptr,
                                        size_t num_bytes)
{
    MDB_val mdb_data_val = { num_bytes, data_ptr };

    if (fpuDbPutItem(serial_number, subkey, mdb_data_val) == 0)
    {
        return true;
    }
    return false;
}

//------------------------------------------------------------------------------
bool ProtectionDbTxn::fpuDbReadRawItem(const char serial_number[],
                                       const char subkey[],
                                       void **data_ptr_ret,
                                       size_t &num_bytes_ret)
{
    MDB_val mdb_data_val;

    if (fpuDbGetItem(serial_number, subkey, mdb_data_val) == 0)
    {
        *data_ptr_ret = mdb_data_val.mv_data;
        num_bytes_ret = mdb_data_val.mv_size;
        return true;
    }
    return false;
}

//------------------------------------------------------------------------------
int ProtectionDbTxn::fpuDbPutItem(const char serial_number[],
                                  const char subkey[],
                                  const MDB_val &data_val)
{
    MDB_val key_val = fpuDbCreateKeyVal(serial_number, subkey);
    return mdb_put(txn_ptr, fpu_dbi, &key_val,
                   const_cast<MDB_val *>(&data_val), 0x0);
}

//------------------------------------------------------------------------------
int ProtectionDbTxn::fpuDbGetItem(const char serial_number[],
                                  const char subkey[],
                                  MDB_val &data_val_ret)
{
    MDB_val key_val = fpuDbCreateKeyVal(serial_number, subkey);
    return mdb_get(txn_ptr, fpu_dbi, &key_val, (MDB_val *)&data_val_ret);
}

//------------------------------------------------------------------------------
MDB_val ProtectionDbTxn::fpuDbCreateKeyVal(const char serial_number[],
                                           const char subkey[])
{
    // N.B. Need static persistent key_str below because its raw c_str is
    // pointed to by the returned MDB_val
    // TODO: This is NOT thread-safe (because single static instance) - will
    // this be a problem?
    static std::string key_str;

    // Create ASCII key of form <serial_number><separator><subkey>
    // IMPORTANT: serial_number and subkey must not contain the
    // keystr_separator_char character
    key_str = std::string(serial_number) + keystr_separator_char + subkey;
    return { key_str.size(), const_cast<void *>((const void *)key_str.c_str()) };
}

//------------------------------------------------------------------------------
ProtectionDbTxn::~ProtectionDbTxn()
{
    mdb_txn_commit(txn_ptr);
    
   
    // TODO: Where to call mdb_env_sync() to flush to disk? Or is this done
    // automatically due to the LMDB config macros / settings?
}


//==============================================================================
// Unit test functions follow
//==============================================================================

static bool protectionDB_TestWithStayingOpen(const std::string &dir_str);
static bool protectionDB_TestWithClosingReopening(const std::string &dir_str);
static bool protectionDB_TestSingleFpuCountersWriting(ProtectionDB &protectiondb);
static bool protectionDB_TestSingleItemWriteRead(ProtectionDB &protectiondb);
static bool protectionDB_TestMultipleItemWriteReads(ProtectionDB &protectiondb);
static std::string getNextFpuTestSerialNumber();


//------------------------------------------------------------------------------
bool protectionDB_Test()
{
    // Performs a suite of protection database tests - reading and writing
    // items within individual or multiple transactions, also with some 
    // database closing/re-opening
    // NOTE: An LMDB database must already exist in dir_str location (see below)
    
    std::string dir_str = "/moonsdata/fpudb_NEWFORMAT";
    bool result_ok = false;
    
    result_ok = protectionDB_TestWithStayingOpen(dir_str);
    
    if (result_ok)
    {
        result_ok = protectionDB_TestWithClosingReopening(dir_str);
    }
    
    return result_ok;
}

//------------------------------------------------------------------------------
static bool protectionDB_TestWithStayingOpen(const std::string &dir_str)
{
    // Performs various ProtectionDB tests with the database being kept open
    // NOTE: An LMDB database must already exist in dir_str location

    ProtectionDB protectiondb;
    bool result_ok = false;
   
    if (protectiondb.open(dir_str))
    {
        result_ok = protectionDB_TestSingleFpuCountersWriting(protectiondb);
        
        if (result_ok)
        {
            result_ok = protectionDB_TestSingleItemWriteRead(protectiondb);
        }

        if (result_ok)
        {
            result_ok = protectionDB_TestMultipleItemWriteReads(protectiondb);
        }
    }
    
    return result_ok;
}

//------------------------------------------------------------------------------
static bool protectionDB_TestWithClosingReopening(const std::string &dir_str)
{
    // Performs various ProtectionDB tests with the database being closed
    // and re-opened between each test
    // NOTE: An LMDB database must already exist in dir_str location
    
    bool result_ok = false;

    // N.B. ProtectionDB is opened (and then closed automatically again) inside
    // each scope
    
    {
        ProtectionDB protectiondb;
        if (protectiondb.open(dir_str))
        {
            result_ok = protectionDB_TestSingleFpuCountersWriting(protectiondb);
        }
        else
        {
            result_ok = false;
        }
    }
    
    // NOTE: If single-stepping through this function, can now do a dump of the
    // database to check its contents, using e.g.: mdb_dump . -p -s fpu

    if (result_ok)
    {
        ProtectionDB protectiondb;
        if (protectiondb.open(dir_str))
        {
            result_ok = protectionDB_TestSingleItemWriteRead(protectiondb);
        }
        else
        {
            result_ok = false;
        }
    }

    // NOTE: If single-stepping through this function, can now do a dump of the
    // database to check its contents, using e.g.: mdb_dump . -p -s fpu

    if (result_ok)
    {
        ProtectionDB protectiondb;
        if (protectiondb.open(dir_str))
        {
            result_ok = protectionDB_TestMultipleItemWriteReads(protectiondb);
        }
        else
        {
            result_ok = false;
        }
    }
    
    return result_ok;    
}

//------------------------------------------------------------------------------
static bool protectionDB_TestSingleFpuCountersWriting(ProtectionDB &protectiondb)
{
    // Tests writing the counters for a single FPU
    
    // TODO: Also add reading back of the counters and comparing against what
    // was written (N.B. Need to populate the counters with some random values)
    
    bool result_ok = false;
    
    auto transaction = protectiondb.createTransaction();
    if (transaction)
    {
        FpuCounters fpu_counters;
        std::string serial_number_str = getNextFpuTestSerialNumber();
        result_ok = transaction->fpuDbPutCounters(serial_number_str.c_str(),
                                                  fpu_counters);
    }
    
    return result_ok;
}

//------------------------------------------------------------------------------
static bool protectionDB_TestSingleItemWriteRead(ProtectionDB &protectiondb)
{
    // Tests writing of a single item and reading it back, all in one transaction
    
    bool result_ok = false;
    
    auto transaction = protectiondb.createTransaction();
    if (transaction)
    {
        // Write item
        std::string serial_number_str = getNextFpuTestSerialNumber();
        char subkey[] = "TestSubkey";
        char data_str[] = "0123456789";
        result_ok = transaction->fpuDbWriteRawItem(serial_number_str.c_str(),
                                                   subkey, (void *)data_str,
                                                   strlen(data_str));

        // Read item back
        void *data_returned_ptr = nullptr;
        size_t num_bytes_returned = 0;
        if (result_ok)
        {
            result_ok = transaction->fpuDbReadRawItem(serial_number_str.c_str(),
                                                      subkey, &data_returned_ptr,
                                                      num_bytes_returned);
        }
        
        // Verify that item read back matches item written
        if (result_ok)
        {
            if ((num_bytes_returned != strlen(data_str)) ||
                (memcmp(data_str, data_returned_ptr, strlen(data_str)) != 0))
            {
                result_ok = false;
            }
        }
    }
    
    return result_ok;
}

//------------------------------------------------------------------------------
static bool protectionDB_TestMultipleItemWriteReads(ProtectionDB &protectiondb)
{
    // Tests writing of multiple items in a first transaction, and reading them
    // back in a second transaction

    bool result_ok = false;
    const int num_iterations = 100;
    std::string serial_number_str = getNextFpuTestSerialNumber();
    uint64_t test_multiplier = 0x123456789abcdef0L;
    
    // N.B. The transactions in the following code are each in their own scope
    // so that writes should be automatically committed when transaction goes out
    // of scope and is destroyed
    
    {
        auto transaction = protectiondb.createTransaction();
        if (transaction)
        {
            result_ok = true;
            for (int i = 0; i < num_iterations; i++)
            {
                char subkey_str[10];
                snprintf(subkey_str, sizeof(subkey_str), "%03d", i);
                uint64_t test_val = ((uint64_t)i) * test_multiplier;
                if (!transaction->fpuDbWriteRawItem(serial_number_str.c_str(),
                                                    subkey_str, (void *)&test_val,
                                                    sizeof(test_val)))
                {
                    // Error
                    result_ok = false;
                    break;
                }
            }
        }
    }

    if (result_ok)
    {
        auto transaction = protectiondb.createTransaction();
        if (transaction)
        {
            result_ok = true;
            for (int i = 0; i < num_iterations; i++)
            {
                char subkey_str[10];
                snprintf(subkey_str, sizeof(subkey_str), "%03d", i);
                uint64_t test_val = ((uint64_t)i) * test_multiplier;
                void *data_returned_ptr = nullptr;
                size_t num_bytes_returned = 0;
                if (transaction->fpuDbReadRawItem(serial_number_str.c_str(),
                                                  subkey_str, &data_returned_ptr,
                                                  num_bytes_returned))
                {
                    if ((num_bytes_returned != sizeof(test_val)) ||
                        (memcmp(data_returned_ptr, (void *)&test_val,
                                sizeof(test_val)) != 0))
                    {
                        result_ok = false;
                        break;
                    }
                }
                else
                {
                    result_ok = false;
                    break;
                }
            }
        }
    }
    
    return result_ok;
}

//------------------------------------------------------------------------------
static std::string getNextFpuTestSerialNumber()
{
    // Provides incrementing serial number strings of the form "TestNNNN", with
    // the NNNN values being incrementing leading-zero values starting from an
    // initial random number 0-4999
    
    // Initialise random number generator
    time_t t;
    srand((unsigned)time(&t));
   
    static int number = rand() % 5000; // 0-4999
    number++;
    
    char serial_number_c_str[20];
    snprintf(serial_number_c_str, sizeof(serial_number_c_str), "Test%04d", number);

    return std::string(serial_number_c_str);
}

//------------------------------------------------------------------------------
