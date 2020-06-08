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
// TODO: Check if "verification" sub-database is needed
static const char *fpu_subdb_name = "fpu";
static const char *healthlog_subdb_name = "healthlog";
static const char *verification_subdb_name = "verification";

// TODO: Put meaningful comment here describing what the strings below are
static const char *dbname_keystr = "fpu";
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
static const char *counters_keystr = "counters";
static const char *serialnumber_used_keystr = "serialnumber_used";

// Character to separate the key/subkey parts of the overall key strings
static const char *keystr_separator_char = "#";

// -----------------------------------------------------------------------------

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

// -----------------------------------------------------------------------------

static MDB_val createFpuDbKeyVal(const char serial_number[],
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
    return { key_str.size(), (void *)key_str.c_str() };
}

static int putFpuDbItem(MDB_txn *txn_ptr, MDB_dbi dbi,
                        const char serial_number[],
                        const char subkey[], const MDB_val &data_val)
{
    MDB_val key_val = createFpuDbKeyVal(serial_number, subkey);
    return mdb_put(txn_ptr, dbi, &key_val, (MDB_val *)&data_val, 0x0);
}

static int getFpuDbItem(MDB_txn *txn_ptr, MDB_dbi dbi,
                        const char serial_number[],
                        const char subkey[], MDB_val &data_val_ret)
{
    MDB_val key_val = createFpuDbKeyVal(serial_number, subkey);
    return mdb_get(txn_ptr, dbi, &key_val, (MDB_val *)&data_val_ret);
}

// -----------------------------------------------------------------------------

ProtectionDB::FpuDbTxn::FpuDbTxn(MDB_env *protectiondb_mdb_env_ptr,
                                 bool &created_ok_ret)
{
    mdb_env_ptr = protectiondb_mdb_env_ptr;

    created_ok_ret = false;
    if (mdb_txn_begin(mdb_env_ptr, nullptr, 0x0, &txn_ptr) == 0)
    {
        // Set dbi to the "fpu" sub-database within the database file (and
        // create sub-database if doesn't already exist)
        if (mdb_dbi_open(txn_ptr, fpu_subdb_name, MDB_CREATE, &dbi) == 0)
        {
            created_ok_ret = true;
        }
    }
}

bool ProtectionDB::FpuDbTxn::putCounters(const char serial_number[],
                                         const FpuCounters &fpu_counters)
{
    MDB_val data_val;
    
    data_val.mv_data = fpu_counters.getRawData(data_val.mv_size); 
   
    if (putFpuDbItem(txn_ptr, dbi, serial_number, counters_keystr,
                     data_val) == 0)
    {
        return true;
    }
    return false;
}

bool ProtectionDB::FpuDbTxn::test_WriteRawItem(const char serial_number[],
                                               const char subkey[],
                                               void *data_ptr, size_t num_bytes)
{
    MDB_val mdb_data_val = { num_bytes, data_ptr };

    if (putFpuDbItem(txn_ptr, dbi, serial_number, subkey, mdb_data_val) == 0)
    {
        return true;
    }
    return false;
}

bool ProtectionDB::FpuDbTxn::test_ReadRawItem(const char serial_number[],
                                              const char subkey[],
                                              void **data_ptr_ret,
                                              size_t &num_bytes_ret)
{
    MDB_val mdb_data_val;

    if (getFpuDbItem(txn_ptr, dbi, serial_number, subkey, mdb_data_val) == 0)
    {
        *data_ptr_ret = mdb_data_val.mv_data;
        num_bytes_ret = mdb_data_val.mv_size;
        return true;
    }
    return false;
}

ProtectionDB::FpuDbTxn::~FpuDbTxn()
{
    mdb_txn_commit(txn_ptr);
    
    // TODO: mdb_dbi_close() has caveats - see its documentation - but this
    // should be OK here?
    mdb_dbi_close(mdb_env_ptr, dbi);
    
    // TODO: Where to call mdb_env_sync() to flush to disk? Or is this done
    // automatically due to the LMDB config macros / settings?
}

// -----------------------------------------------------------------------------

bool ProtectionDB::open(const std::string &dir_str)
{
    // TODO: Must only call exactly once for a particular LMDB file in this
    // process (see the LMDB documentation) - enforce this somehow?
    
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

    // TODO: In the original Python code, the following are defaulted - see
    // https://lmdb.readthedocs.io/en/release/ -> Environment class - 
    // max_spare_txns=1
    // create=True      <== **IMPORTANT** - create directory if doesn't exist?

    int mdb_result = mdb_env_create(&mdb_env_ptr);
    if (mdb_result == 0)
    {
        mdb_result = mdb_env_set_maxdbs(mdb_env_ptr, 10);
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
            // TODO: Check the required state of MDB_NOTLS flag
            unsigned int flags = 0x0;
            mdb_result = mdb_env_open(mdb_env_ptr, dir_str.c_str(), flags, 0755);
        }
    }

    if (mdb_result != 0)
    {
        // Failure - call mdb_env_close() to discard the MDB_env handle
        mdb_env_close(mdb_env_ptr);
        mdb_env_ptr = nullptr;
    }

    if (mdb_env_ptr != nullptr)
    {
        return true;
    }
    return false;
}

std::unique_ptr<ProtectionDB::FpuDbTxn> ProtectionDB::createFpuDbTransaction()
{
    std::unique_ptr<ProtectionDB::FpuDbTxn> ptr_returned;

    if (mdb_env_ptr != nullptr)
    {
        bool created_ok = false;
        // TODO: C++11 doesn't support make_unique - ask if OK to compile
        // as C++14 in final ESO driver
        //ptr_returned = std::make_unique<FpuDbTxn>(*_mdb_env_ptr, created_ok);
        ptr_returned.reset(new FpuDbTxn(mdb_env_ptr, created_ok));
        if (!created_ok)
        {
            ptr_returned.reset();
        }
    }
    return std::move(ptr_returned);
}

std::unique_ptr<ProtectionDB::HealthLogTxn> ProtectionDB::createHealthLogDbTransaction()
{
    // TODO
}
 
ProtectionDB::~ProtectionDB()
{
    // TODO: Release all handles, close ProtectionDB LMDB environment etc
    mdb_env_close(mdb_env_ptr);
    
    // TODO: Call any others? e.g. 
    // mdb_env_sync()?
    // Any others?
    
}


// *****************************************************************************
// *****************************************************************************
// Unit test functions follow
// *****************************************************************************
// *****************************************************************************

static bool protectionDB_TestWithStayingOpen(const std::string &dir_str);
static bool protectionDB_TestWithClosingReopening(const std::string &dir_str);
static bool protectionDB_TestSingleFpuCountersWriting(ProtectionDB &protectiondb);
static bool protectionDB_TestSingleItemWriteRead(ProtectionDB &protectiondb);
static bool protectionDB_TestMultipleItemWriteReads(ProtectionDB &protectiondb);
static std::string getNextFpuTestSerialNumber();


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

static bool protectionDB_TestSingleFpuCountersWriting(ProtectionDB &protectiondb)
{
    // Tests writing the counters for a single FPU
    
    // TODO: Also add reading back of the counters and comparing against what
    // was written (N.B. Need to populate the counters with some random values)
    
    bool result_ok = false;
    
    auto fpudb_txn = protectiondb.createFpuDbTransaction();
    if (fpudb_txn)
    {
        FpuCounters fpu_counters;
        std::string serial_number_str = getNextFpuTestSerialNumber();
        result_ok = fpudb_txn->putCounters(serial_number_str.c_str(),
                                           fpu_counters);
    }
    
    return result_ok;
}

static bool protectionDB_TestSingleItemWriteRead(ProtectionDB &protectiondb)
{
    // Tests writing of a single item and reading it back, all in one transaction
    
    bool result_ok = false;
    
    auto fpudb_txn = protectiondb.createFpuDbTransaction();
    if (fpudb_txn)
    {
        // Write item
        std::string serial_number_str = getNextFpuTestSerialNumber();
        const char subkey[] = "TestSubkey";
        const char data_str[] = "0123456789";
        result_ok = fpudb_txn->test_WriteRawItem(serial_number_str.c_str(),
                                                 subkey, (void *)data_str,
                                                 strlen(data_str));

        // Read item back
        void *data_returned_ptr = nullptr;
        size_t num_bytes_returned = 0;
        if (result_ok)
        {
            result_ok = fpudb_txn->test_ReadRawItem(serial_number_str.c_str(),
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

static bool protectionDB_TestMultipleItemWriteReads(ProtectionDB &protectiondb)
{
    // Tests writing of multiple items in a first transaction, and reading them
    // back in a second transaction

    bool result_ok = false;
    const int num_iterations = 100;
    std::string serial_number_str = getNextFpuTestSerialNumber();
    uint64_t test_multiplier = 0x123456789abcdef0L;
    
    // N.B. The transactions in the following code are each in their own scope
    // so that writes should be automatically committed when fpudb_txn goes out
    // of scope and is destroyed
    
    {
        auto fpudb_txn = protectiondb.createFpuDbTransaction();
        if (fpudb_txn)
        {
            result_ok = true;
            for (int i = 0; i < num_iterations; i++)
            {
                char subkey_str[10];
                snprintf(subkey_str, sizeof(subkey_str), "%03d", i);
                uint64_t test_val = ((uint64_t)i) * test_multiplier;
                if (!fpudb_txn->test_WriteRawItem(serial_number_str.c_str(),
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
        auto fpudb_txn = protectiondb.createFpuDbTransaction();
        if (fpudb_txn)
        {
            result_ok = true;
            for (int i = 0; i < num_iterations; i++)
            {
                char subkey_str[10];
                snprintf(subkey_str, sizeof(subkey_str), "%03d", i);
                uint64_t test_val = ((uint64_t)i) * test_multiplier;
                void *data_returned_ptr = nullptr;
                size_t num_bytes_returned = 0;
                if (fpudb_txn->test_ReadRawItem(serial_number_str.c_str(),
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


// -----------------------------------------------------------------------------
// TODO: Old experimentation code - delete once happy with new code above
#if 0
// -----------------------------------------------------------------------------

int ProtectionDB_OLD::doStuff()
{
    int major = 1;
    int minor = 2;
    int patch = 3;
    char *lmdb_version_str = mdb_version(&major, &minor, &patch);

    return 456;
}

bool ProtectionDB_OLD::getRawField(MDB_txn &txn, MDB_dbi dbi,
                                   const char serial_number[],
                                   const char subkey[], MDB_val &data_val_ret)
{
    std::string key_str = std::string(serial_number) + keystr_separator_char +
                          subkey;
    MDB_val key_val = { key_str.size(), (void *)key_str.c_str() };

    int mdb_result = mdb_get(&txn, dbi, &key_val, &data_val_ret);
    
    if (mdb_result == 0)
    {
        return true;
    }
    return false;
}

void ProtectionDB_OLD::putField(MDB_txn &txn, MDB_dbi dbi, const char serial_number[],
                                const char subkey[], MDB_val &data_val)
{
    // Create ASCII key of form <serial_number><separator><subkey>
    // IMPORTANT: serial_number and subkey must not contain the
    // keystr_separator_char character
    std::string key_str = std::string(serial_number) + keystr_separator_char +
                          subkey;
    
    MDB_val key_val = { key_str.size(), (void *)key_str.c_str() };

    // TODO: Check if final flags argument below is OK
    int mdb_result = mdb_put(&txn, dbi, &key_val, &data_val, 0x0);

    // TODO: Call mdb_txn_commit() here?? Check against the Python version
    // AND where to call mdb_env_sync() to flush to disk?
    
    // TODO: Return a result value
}

void ProtectionDB_OLD::putCounters(MDB_txn &txn, MDB_dbi dbi, 
                                   const char serial_number[],
                                   const FpuCounters &fpu_counters)
{
    MDB_val data_val;
    
    data_val.mv_data = fpu_counters.getRawData(data_val.mv_size); 

    putField(txn, dbi, serial_number, counters_keystr, data_val);

    // TODO: Return a result value
}

MDB_env *protectionDB_Open_OLD(const std::string &dir_str)
{
    // TODO: Must only call exactly once for a particular LMDB file in this
    // process (see the LMDB documentation) - enforce this somehow?
    
    MDB_env *env_ptr = nullptr;
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

    //.......................
    // TODO: Python to c++ conversion WIP
    // See https://lmdb.readthedocs.io/en/release/ -> Environment class
    // env = lmdb.open(database_file_name, max_dbs=10, map_size=dbsize)

    // TODO: In the original Python code, the following are defaulted - see
    // https://lmdb.readthedocs.io/en/release/ -> Environment class - 
    // max_spare_txns=1
    // create=True      <== **IMPORTANT** - create directory if doesn't exist?

    int mdb_result = mdb_env_create(&env_ptr);
    if (mdb_result == 0)
    {
        mdb_result = mdb_env_set_maxdbs(env_ptr, 10);
        if (mdb_result == 0)
        {
            mdb_result = mdb_env_set_mapsize(env_ptr, dbsize);
        }

        if (mdb_result == 0)
        {
            // TODO: The following keeps compatibility with the original
            // Python code defaults, but are this many readers needed?
            mdb_result = mdb_env_set_maxreaders(env_ptr, 126);
        }

        if (mdb_result == 0)
        {
            // N.B. Using default flags for now, so flags value is 0x0
            // TODO: Check the required state of MDB_NOTLS flag
            unsigned int flags = 0x0;
            mdb_result = mdb_env_open(env_ptr, dir_str.c_str(),
                                      flags, 0755);
        }

        if (mdb_result != 0)
        {
            // Failure - call mdb_env_close() to discard the MDB_env handle
            mdb_env_close(env_ptr);
            env_ptr = nullptr;
        }
    }

    //.......................

    // TODO: Return mdb_result somehow as well?

    return env_ptr;
}

// N.B. C++ replacement for Python open_database_env()
MDB_env *protectionDB_OpenEnv_OLD(bool mockup)
{
    // TODO: Must only call exactly once for a particular LMDB file in this
    // process (see the LMDB documentation) - enforce this somehow?
    MDB_env *env_ptr = nullptr;
    
    std::string dir_str = protectionDB_GetDirFromLinuxEnv(mockup);
    if (!dir_str.empty())
    {
        env_ptr = protectionDB_Open_OLD(dir_str);
    }
        
    return env_ptr;
}            

void protectionDB_Test_OLD()
{
    // Initial ad-hoc test function - single-step through and look at results
    // N.B. An LMDB database (consisting of data.mdb + lock.mdb files) must
    // already exist in the protectiondb_dir directory location specified below 
    
    int mdb_result;
    ProtectionDB_OLD protectionDB;
    
    protectionDB.doStuff();
    
    std::string protectiondb_dir = "/moonsdata/fpudb_NEWFORMAT";
    MDB_env *env_ptr = protectionDB_OpenEnv_OLD(true);

    MDB_txn *txn_ptr;
    MDB_dbi dbi;

    const char *test_serialnumber_str = "BWTest0001";
    const char *test_subkey_str = "BWTestSubkey";

    //..........................................................................

    mdb_result = mdb_txn_begin(env_ptr, nullptr, 0x0, &txn_ptr);
    
    // Set dbi to the "fpu" sub-database within the database file (and create
    // sub-database if doesn't already exist)
    mdb_result = mdb_dbi_open(txn_ptr, "fpu", MDB_CREATE, &dbi);
    
    // Write a test record to the fpu sub-database
    const char *test_str = "abc";
    MDB_val data_val_write = { strlen(test_str), (void *)test_str };
    protectionDB.putField(*txn_ptr, dbi, test_serialnumber_str, test_subkey_str,
                          data_val_write);
    
    mdb_result = mdb_txn_commit(txn_ptr);
    
    //..........................................................................

    txn_ptr = nullptr;

    mdb_result = mdb_txn_begin(env_ptr, nullptr, 0x0, &txn_ptr);
    
    // Set dbi to the "fpu" sub-database within the database file (and create
    // sub-database if doesn't already exist)
    mdb_result = mdb_dbi_open(txn_ptr, "fpu", MDB_CREATE, &dbi);
    
    MDB_val data_val_read;
    bool worked_ok = protectionDB.getRawField(*txn_ptr, dbi,
                                              test_serialnumber_str,
                                              test_subkey_str, data_val_read);

    // ****** IMPORTANT NOTE: "Values returned from the database are valid only
    // until a subsequent update operation, or the end of the transaction"
    
    mdb_result = mdb_txn_commit(txn_ptr);

    //..........................................................................

    // TODO: Release and close all necessary stuff
    // mdb_env_sync()?
    // mdb_dbi_close()?
    // mdb_env_close()?
    // Any others?
}

// -----------------------------------------------------------------------------
#endif // 0
// -----------------------------------------------------------------------------

