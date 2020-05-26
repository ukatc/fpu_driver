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
#include <cstring>
#include "ProtectionDB.h"

static const char *dbname = "fpu";
static const char *alpha_positions = "apos";
static const char *beta_positions = "bpos";
static const char *waveform_table = "wtab";
static const char *waveform_reversed = "wf_reversed";
static const char *alpha_limits = "alimits";
static const char *beta_limits = "blimits";
static const char *free_beta_retries = "bretries";
static const char *beta_retry_count_cw = "beta_retry_count_cw";
static const char *beta_retry_count_acw = "beta_retry_count_acw"; 
static const char *free_alpha_retries = "aretries";
static const char *alpha_retry_count_cw = "alpha_retry_count_cw";
static const char *alpha_retry_count_acw = "alpha_retry_count_acw";
static const char *counters = "counters2";
static const char *serialnumber_used = "serialnumber_used";

int ProtectionDB::doStuff()
{
    int major = 1;
    int minor = 2;
    int patch = 3;
    char *lmdb_version_str = mdb_version(&major, &minor, &patch);

    return 456;
}


MDB_env *open_database_env(bool mockup)
{
    // TODO: Check if nullptr is OK to use for getenv() returning NULL
    
    // TODO: Must only call exactly once for a particular LMDB file in this
    // process (see the LMDB documentation) - enforce this somehow?
    
    // TODO: If database_file_name and file_name_ptr actually specify directories
    // rather than filenames, then change their names to _dir
    
    MDB_env *env_ptr = nullptr;
    char *file_name_ptr = nullptr;
    std::string database_file_name;
    
    if (mockup)
    {
        file_name_ptr = getenv("FPU_DATABASE_MOCKUP");
        if (file_name_ptr == nullptr)
        {
            file_name_ptr = getenv("FPU_DATABASE");
            if (file_name_ptr != nullptr)
            {
                database_file_name = file_name_ptr;
                database_file_name += "_mockup";
            }
        }
        else
        {
            database_file_name = file_name_ptr;
        }
    }
    else
    {
        // A good value is "/var/lib/fpudb"
        file_name_ptr = getenv("FPU_DATABASE");
        if (file_name_ptr != nullptr)
        {
            database_file_name = file_name_ptr;
        }
    }

    if (database_file_name != "")
    {
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
        
        int mdb_result;
	mdb_result = mdb_env_create(&env_ptr);
        if (mdb_result == 0)
        {
            mdb_result = mdb_env_set_mapsize(env_ptr, dbsize);
            if (mdb_result == 0)
            {
                // TODO: The following keeps compatibility with the original
                // Python code defaults, but are this many readers needed?
                mdb_result = mdb_env_set_maxreaders(env_ptr, 126);
            }
            
            if (mdb_result == 0)
            {
                mdb_result = mdb_env_set_maxdbs(env_ptr, 10);
            }
            
            if (mdb_result == 0)
            {
                // N.B. Using default flags for now, so flags value is 0x0
                // TODO: Check the required state of MDB_NOTLS flag
                unsigned int flags = 0x0;
                mdb_result = mdb_env_open(env_ptr,
                                          database_file_name.c_str(),
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
    }

    return env_ptr;
}


