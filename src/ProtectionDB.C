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
#include <string>
#include <cstring>
#include "ProtectionDB.h"

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
static const char *counters_keystr = "counters2";
static const char *serialnumber_used_keystr = "serialnumber_used";

static const char *keystr_separator_char = "#";

// -----------------------------------------------------------------------------

static std::string protectiondb_get_dir_from_linux_env(bool mockup);
static MDB_env *protectiondb_open_env(const std::string &dir_str);

// -----------------------------------------------------------------------------


int ProtectionDB::doStuff()
{
    int major = 1;
    int minor = 2;
    int patch = 3;
    char *lmdb_version_str = mdb_version(&major, &minor, &patch);

    return 456;
}


bool ProtectionDB::getRawField(MDB_txn &txn, MDB_dbi dbi,
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


void ProtectionDB::putField(MDB_txn &txn, MDB_dbi dbi, const char serial_number[],
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


void ProtectionDB::put_counters(MDB_txn &txn, MDB_dbi dbi, 
                                const char serial_number[],
                                const FpuCounters &fpu_counters)
{
    MDB_val data_val;
    
    data_val.mv_data = fpu_counters.getRawData(data_val.mv_size); 

    putField(txn, dbi, serial_number, counters_keystr, data_val);

    // TODO: Return a result value
}


// -----------------------------------------------------------------------------

MDB_env *open_database_env(bool mockup)
{
    // TODO: Must only call exactly once for a particular LMDB file in this
    // process (see the LMDB documentation) - enforce this somehow?
    MDB_env *env_ptr = nullptr;
    
    std::string dir_str = protectiondb_get_dir_from_linux_env(mockup);
    if (!dir_str.empty())
    {
        env_ptr = protectiondb_open_env(dir_str);
    }
        
    return env_ptr;
}            


std::string protectiondb_get_dir_from_linux_env(bool mockup)
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

    
MDB_env *protectiondb_open_env(const std::string &dir_str)
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


void protectiondb_test()
{
    // Initial ad-hoc test function - single-step through and look at results
    // N.B. An LMDB database (consisting of data.mdb + lock.mdb files) must
    // already exist in the protectiondb_dir directory location specified below 
    
    int mdb_result;
    ProtectionDB protectionDB;
    
    protectionDB.doStuff();
    
    std::string protectiondb_dir = "/moonsdata/fpudb_NEWFORMAT";
    MDB_env *env_ptr = protectiondb_open_env(protectiondb_dir);

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

