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

// Database key strings
static const char *alpha_position_keystr = "apos";
static const char *beta_position_keystr = "bpos";
static const char *waveform_table_keystr = "wtable";
static const char *waveform_reversed_keystr = "wfreversed";
static const char *alpha_limit_keystr = "alimit";
static const char *beta_limit_keystr = "blimit";
static const char *free_alpha_retries_keystr = "aretries";
static const char *alpha_retries_cw_keystr = "aretries_cw";
static const char *alpha_retries_acw_keystr = "aretries_acw";
static const char *free_beta_retries_keystr = "bretries";
static const char *beta_retries_cw_keystr = "bretries_cw";
static const char *beta_retries_acw_keystr = "bretries_acw";
static const char *counters_keystr = "counters";
// TODO: Will the following string be used - is there a corresponding string
// used in the Python version?
//static const char *serialnumber_used_keystr = "serialnumber_used";

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

#ifdef FPU_DB_DATA_AGGREGATED
//------------------------------------------------------------------------------
bool ProtectionDbTxn::fpuDbTransferFpu(DbTransferType transfer_type,
                                       const char serial_number[],
                                       FpuDbData &fpu_db_data)
{

    // TODO: Test this function properly - only visually checked so far

    ///*************** TODO: What to do with the offset values??
    double datum_offset;

    bool result_ok = fpuDbTransferPosition(transfer_type, 
                                           FpuDbPositionType::AlphaPos,
                                           serial_number, fpu_db_data.apos,
                                           datum_offset);

    if (result_ok)
    {
        result_ok = fpuDbTransferPosition(transfer_type, 
                                          FpuDbPositionType::BetaPos,
                                          serial_number, fpu_db_data.bpos,
                                          datum_offset);
    }

    if (result_ok)
    {
        result_ok = fpuDbTransferWfReversedFlag(transfer_type, serial_number,
                                                fpu_db_data.wf_reversed);
    }

    if (result_ok)
    {
        result_ok = fpuDbTransferPosition(transfer_type, 
                                          FpuDbPositionType::AlphaLimit,
                                          serial_number, fpu_db_data.alimits,
                                          datum_offset);
    }

    if (result_ok)
    {
        result_ok = fpuDbTransferPosition(transfer_type, 
                                          FpuDbPositionType::BetaLimit,
                                          serial_number, fpu_db_data.blimits,
                                          datum_offset);
    }

    // TODO: Does FreeAlphaRetries / FreeBetaRetries actually correspond to
    // maxrareties/maxbretries in the items below?

    if (result_ok)
    {
        result_ok = fpuDbTransferInt64Val(transfer_type,
                                          FpuDbIntValType::FreeAlphaRetries,
                                          serial_number, 
                                          fpu_db_data.maxaretries);
    }

    if (result_ok)
    {
        result_ok = fpuDbTransferInt64Val(transfer_type,
                                          FpuDbIntValType::AlphaRetries_CW,
                                          serial_number, 
                                          fpu_db_data.aretries_cw);
    }

    if (result_ok)
    {
        result_ok = fpuDbTransferInt64Val(transfer_type,
                                          FpuDbIntValType::AlphaRetries_ACW,
                                          serial_number, 
                                          fpu_db_data.aretries_acw);
    }

    if (result_ok)
    {
        result_ok = fpuDbTransferInt64Val(transfer_type,
                                          FpuDbIntValType::FreeBetaRetries,
                                          serial_number, 
                                          fpu_db_data.maxbretries);
    }

    if (result_ok)
    {
        result_ok = fpuDbTransferInt64Val(transfer_type,
                                          FpuDbIntValType::BetaRetries_CW,
                                          serial_number, 
                                          fpu_db_data.bretries_cw);
    }

    if (result_ok)
    {
        result_ok = fpuDbTransferInt64Val(transfer_type,
                                          FpuDbIntValType::BetaRetries_ACW,
                                          serial_number, 
                                          fpu_db_data.bretries_acw);
    }

    if (result_ok)
    {
        result_ok = fpuDbTransferCounters(transfer_type, serial_number,
                                          fpu_db_data.counters);
    }

    if (result_ok)
    {
        result_ok = fpuDbTransferWaveform(transfer_type, serial_number,
                                          fpu_db_data.waveform);
    }

    return result_ok;
}
#endif // FPU_DB_DATA_AGGREGATED

//------------------------------------------------------------------------------
bool ProtectionDbTxn::fpuDbTransferPosition(DbTransferType transfer_type,
                                            FpuDbPositionType position_type,
                                            const char serial_number[],
                                            Interval &interval,
                                            double &datum_offset)
{
    // Reads or writes an aggregate position type (interval + datum offset)
    // of type FpuDbPositionType for an FPU
    
    static const struct
    {
        FpuDbPositionType type;
        const char *subkey;
    } position_subkeys[(int)FpuDbPositionType::NumTypes] = 
    {
        { FpuDbPositionType::AlphaLimit,  alpha_limit_keystr    },
        { FpuDbPositionType::AlphaPos,    alpha_position_keystr },
        { FpuDbPositionType::BetaLimit,   beta_limit_keystr     }, 
        { FpuDbPositionType::BetaPos,     beta_position_keystr  }
    };

    const char *subkey = nullptr;
    for (int i = 0; i < (int)FpuDbPositionType::NumTypes; i++)
    {
        if (position_subkeys[i].type == position_type)
        {
            subkey = position_subkeys[i].subkey;
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
            if (fpuDbWriteItem(serial_number, subkey, doubles_array,
                               sizeof(doubles_array)))
            {
                return true;
            }
        }
        else
        {
            // Read the position item
            void *item_data_ptr = nullptr;
            int num_item_data_bytes = 0;
            if (fpuDbGetItemDataPtrAndSize(serial_number, subkey,
                                           &item_data_ptr, num_item_data_bytes))
            {
                if (num_item_data_bytes == sizeof(doubles_array))
                {
                    memcpy(doubles_array, item_data_ptr, sizeof(doubles_array));
                    interval = Interval(doubles_array[0], doubles_array[1]);
                    datum_offset = doubles_array[2];
                    return true;
                }
            }
        }
    }

    return false;
}

//------------------------------------------------------------------------------
bool ProtectionDbTxn::fpuDbTransferCounters(DbTransferType transfer_type,
                                            const char serial_number[],
                                            FpuCounters &fpu_counters)
{
    // Reads or writes a set of FPU counters
    
    if (transfer_type == DbTransferType::Write)
    {
        if (fpuDbWriteItem(serial_number, counters_keystr,
                           fpu_counters.getRawBytesPtr(),
                           fpu_counters.getNumRawBytes()))
        {
            return true;
        }
    }
    else
    {
        void *item_data_ptr = nullptr;
        int num_item_bytes = 0;
        if (fpuDbGetItemDataPtrAndSize(serial_number, counters_keystr,
                                       &item_data_ptr, num_item_bytes))
        {
            if (num_item_bytes == fpu_counters.getNumRawBytes())
            {
                fpu_counters.populateFromRawBytes(item_data_ptr);
                return true;
            }
        }
    }

    return false;
}

//------------------------------------------------------------------------------
bool ProtectionDbTxn::fpuDbTransferWaveform(DbTransferType transfer_type,
                                            const char serial_number[],
                                            t_waveform_steps &waveform)
{
    // Reads or writes a forward or reversed waveform

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

        // Write to database
        if (fpuDbWriteItem(serial_number, waveform_table_keystr,
                           bytesBuf.data(), bytesBuf.size()))
        {
            return true;
        }
    }
    else
    {
        void *item_data_ptr = nullptr;
        int num_item_bytes = 0;
        if (fpuDbGetItemDataPtrAndSize(serial_number, waveform_table_keystr,
                                       &item_data_ptr, num_item_bytes))
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
                return true;
            }
            else
            {
                waveform.resize(0);
            }
        }
    }

    return false;
}

//------------------------------------------------------------------------------
bool ProtectionDbTxn::fpuDbTransferInt64Val(DbTransferType transfer_type,
                                            FpuDbIntValType intval_type,
                                            const char serial_number[],
                                            int64_t &int64_val)
{
    // Reads or writes an int64_t value

    static const struct
    {
        FpuDbIntValType type;
        const char *subkey;
    } intval_type_subkeys[(int)FpuDbIntValType::NumTypes] =
    {
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
            if (fpuDbWriteItem(serial_number, subkey, &int64_val,
                               sizeof(int64_t)))
            {
                return true;
            }
        }
        else
        {
            void *item_data_ptr = nullptr;
            int num_item_bytes = 0;
            if (fpuDbGetItemDataPtrAndSize(serial_number, subkey,
                                           &item_data_ptr, num_item_bytes))
            {
                if (num_item_bytes == sizeof(int64_t))
                {
                    memcpy(&int64_val, item_data_ptr, sizeof(int64_t));
                    return true;
                }
            }
        }
    }

    return false;
}

//------------------------------------------------------------------------------
bool ProtectionDbTxn::fpuDbTransferWfReversedFlag(DbTransferType transfer_type,
                                                  const char serial_number[],
                                                  bool &wf_reversed)
{
    // Reads or writes the waveform-reversed bool flag
    
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
        if (fpuDbWriteItem(serial_number, waveform_reversed_keystr,
                           &wf_reversed_byte_val, sizeof(uint8_t)))
        {
            return true;
        }
    }
    else
    {
        void *item_data_ptr = nullptr;
        int num_item_bytes = 0;
        if (fpuDbGetItemDataPtrAndSize(serial_number, waveform_reversed_keystr,
                                       &item_data_ptr, num_item_bytes))
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
                return true;
            }
        }
    }

    return false;
}

//------------------------------------------------------------------------------
bool ProtectionDbTxn::fpuDbWriteItem(const char serial_number[],
                                     const char subkey[], void *data_ptr,
                                     int num_bytes)
{
    // Writes the specified item. If an item with the same serial_number/subkey
    // combination already exists then it is overwritten.
    
    MDB_val key_val = fpuDbCreateKeyVal(serial_number, subkey);
    MDB_val data_val = { (size_t)num_bytes, data_ptr };
    if (mdb_put(txn_ptr, fpu_dbi, &key_val, &data_val, 0x0) == MDB_SUCCESS)
    {
        return true;
    }
    return false;
}

//------------------------------------------------------------------------------
bool ProtectionDbTxn::fpuDbGetItemDataPtrAndSize(const char serial_number[],
                                                 const char subkey[],
                                                 void **data_ptr_ret,
                                                 int &num_bytes_ret)
{
    // For the specified item, gets a pointer to its data and gets its size -
    // this is possible because this is an in-memory database
    
    MDB_val key_val = fpuDbCreateKeyVal(serial_number, subkey);
    MDB_val data_val = { 0, nullptr };
    if (mdb_get(txn_ptr, fpu_dbi, &key_val, &data_val) == MDB_SUCCESS)
    {
        *data_ptr_ret = data_val.mv_data;
        num_bytes_ret = data_val.mv_size;
        return true;
    }
    *data_ptr_ret = nullptr;
    num_bytes_ret = 0;
    return false;
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
    // keystr_separator_char character. *** TODO ***: Check for this here?
    key_str = std::string(serial_number) + keystr_separator_char + subkey;
    return { key_str.size(), const_cast<void *>((const void *)key_str.c_str()) };
}

//------------------------------------------------------------------------------
ProtectionDbTxn::~ProtectionDbTxn()
{
    mdb_txn_commit(txn_ptr);
    
   
    // TODO: Where to call mdb_env_sync() to flush to disk? Or is this done
    // automatically due to the LMDB config macros / settings?
    // NOTE: In FpuGridDriver.py -> GridDriver, is done in a number of places -
    // search for "self.env.sync()"

}

//==============================================================================

