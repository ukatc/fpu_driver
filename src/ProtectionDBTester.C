// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-08-27  Created.
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME ProtectionDBTester.C
//
// Provides test functionality for the ProtectionDB grid driver protection
// database classes.
//
////////////////////////////////////////////////////////////////////////////////

#include <string.h>
#include "ProtectionDBTester.h"

//------------------------------------------------------------------------------
bool ProtectionDBTester::doLoopingTestsWithConsoleOutput()
{
    const int num_iterations = 100;
    for (int i = 1; i <= num_iterations; i++)
    {
        printf("Test #%d of %d: Writing/verifying a single FPU's data items: ",
               i, num_iterations);   // TODO: Use cout / endl stuff instead
        
        if (doTests())
        {
            printf("Passed\n");
        }
        else
        {
            printf("***FAILED***\n");
            return false;
        }
    }

    return true;
}

//------------------------------------------------------------------------------
bool ProtectionDBTester::doTests()
{
    // Performs a suite of protection database tests - reading and writing
    // items within individual or multiple transactions, also with some 
    // database closing/re-opening
    // IMPORTANT NOTES:
    //   - An LMDB database must already exist in dir_str location (see below)
    //   - This test functionality currently generates random FPU serial
    //     numbers (see getRandomFpuTestSerialNumber() which it then uses to
    //     write to and read from the FPU database. If an item with the same
    //     serial number / subkey combination already exists in the database
    //     then it is overwritten

    //**********************
    // TODO: I changed the following directory stuff to consolidate it better
    // with the other places where the database is accessed from (GridDriver
    // usage, FPUAdmin database management tool etc) - check that it's still
    // OK
    //std::string dir_str = "/moonsdata/fpudb_NEWFORMAT";
    const bool mockup = true;
    std::string dir_str = ProtectionDB::getDirFromLinuxEnv(mockup);
    //**********************
    
    bool result_ok = false;

    if (!dir_str.empty())
    {
        result_ok = testFpuDbDataClass();

        if (result_ok)
        {
            result_ok = testWithStayingOpen(dir_str);
        }

        if (result_ok)
        {
            result_ok = testWithClosingReopening(dir_str);
        }
    }
    
    return result_ok;
}

//------------------------------------------------------------------------------
bool ProtectionDBTester::testWithStayingOpen(const std::string &dir_str)
{
    // Performs various ProtectionDB tests with the database being kept open
    // NOTE: An LMDB database must already exist in dir_str location

    ProtectionDB protectiondb;
    MdbResult mdb_result = MDB_PANIC;
   
    if (protectiondb.open(dir_str) == MDB_SUCCESS)
    {
        mdb_result = testFpuSingleItemWriteRead(protectiondb);
        
        if (mdb_result == MDB_SUCCESS)
        {
            mdb_result = testFpuMultipleItemWriteReads(protectiondb);
        }

        if (mdb_result == MDB_SUCCESS)
        {
            mdb_result = testFpuIntervalTransfer(protectiondb);
        }
        
        if (mdb_result == MDB_SUCCESS)
        {
            mdb_result = testFpuCountersTransfer(protectiondb);
        }
        
        if (mdb_result == MDB_SUCCESS)
        {
            mdb_result = testFpuWaveformTransfer(protectiondb);
        }
        
        if (mdb_result == MDB_SUCCESS)
        {
            mdb_result = testFpuInt64ValTransfer(protectiondb);
        }
        
        if (mdb_result == MDB_SUCCESS)
        {
            mdb_result = testFpuWfReversedFlagTransfer(protectiondb);
        }
        
        if (mdb_result == MDB_SUCCESS)
        {
            mdb_result = testFullFpuDataTransfer(protectiondb);
        }
        
        // Run the sync() function
        if (mdb_result == MDB_SUCCESS)
        {
            mdb_result = protectiondb.sync();
        }
    }
    
    if (mdb_result != MDB_SUCCESS)
    {
        return false;
    }
    return true;
}

//------------------------------------------------------------------------------
bool ProtectionDBTester::testWithClosingReopening(const std::string &dir_str)
{
    // Performs various ProtectionDB tests with the database being closed
    // and re-opened between each test,
    // NOTE: An LMDB database must already exist in dir_str location
    
    // Notes:
    //   - Inside each scope, the protection database is opened at the
    //     beginning, and then closed automatically upon exiting the scope
    //   - If single-stepping through this function, then after each database
    //     closure, can do a dump of the database to check its contents, using
    //     e.g.: mdb_dump . -p -s fpu

    MdbResult mdb_result = MDB_PANIC;
    
    {
        ProtectionDB protectiondb;
        mdb_result = protectiondb.open(dir_str);
        if (mdb_result == MDB_SUCCESS)
        {
            mdb_result = testFpuSingleItemWriteRead(protectiondb);
        }
    }
    
    if (mdb_result == MDB_SUCCESS)
    {
        ProtectionDB protectiondb;
        mdb_result = protectiondb.open(dir_str);
        if (mdb_result == MDB_SUCCESS)
        {
            mdb_result = testFpuMultipleItemWriteReads(protectiondb);
        }
    }

    if (mdb_result == MDB_SUCCESS)
    {
        ProtectionDB protectiondb;
        mdb_result = protectiondb.open(dir_str);
        if (mdb_result == MDB_SUCCESS)
        {
            mdb_result = testFpuIntervalTransfer(protectiondb);
        }
    }
    
    if (mdb_result == MDB_SUCCESS)
    {
        ProtectionDB protectiondb;
        mdb_result = protectiondb.open(dir_str);
        if (mdb_result == MDB_SUCCESS)
        {
            mdb_result = testFpuCountersTransfer(protectiondb);
        }
    }
    
    if (mdb_result == MDB_SUCCESS)
    {
        ProtectionDB protectiondb;
        mdb_result = protectiondb.open(dir_str);
        if (mdb_result == MDB_SUCCESS)
        {
            mdb_result = testFpuWaveformTransfer(protectiondb);
        }
    }
    
    if (mdb_result == MDB_SUCCESS)
    {
        ProtectionDB protectiondb;
        mdb_result = protectiondb.open(dir_str);
        if (mdb_result == MDB_SUCCESS)
        {
            mdb_result = testFpuInt64ValTransfer(protectiondb);
        }
    }
    
    if (mdb_result == MDB_SUCCESS)
    {
        ProtectionDB protectiondb;
        mdb_result = protectiondb.open(dir_str);
        if (mdb_result == MDB_SUCCESS)
        {
            mdb_result = testFpuWfReversedFlagTransfer(protectiondb);
        }
    }
    
    if (mdb_result == MDB_SUCCESS)
    {
        ProtectionDB protectiondb;
        mdb_result = protectiondb.open(dir_str);
        if (mdb_result == MDB_SUCCESS)
        {
            mdb_result = testFullFpuDataTransfer(protectiondb);
        }
    }
    
    if (mdb_result != MDB_SUCCESS)
    {
        return false;
    }
    return true;
}

//------------------------------------------------------------------------------
MdbResult ProtectionDBTester::testFpuIntervalTransfer(ProtectionDB &protectiondb)
{
    // Tests writing and reading back an aggregate interval + datum offset
    // of an FPU
    
    MdbResult mdb_result = MDB_PANIC;
    
    auto transaction = protectiondb.createTransaction(mdb_result);
    if (transaction)
    {
        FpuDbIntervalType interval_type = FpuDbIntervalType::BetaLimits;
        Interval interval_write(1.2, 3.4);
        double datum_offset_write = ALPHA_DATUM_OFFSET;
        std::string serial_number_str = getRandomFpuTestSerialNumber();

        // Write position value
        mdb_result = transaction->fpuDbTransferInterval(DbTransferType::Write,
                                                        interval_type,
                                                        serial_number_str.c_str(),
                                                        interval_write,
                                                        datum_offset_write);
        if (mdb_result == MDB_SUCCESS)
        {
            // Read back position value
            Interval interval_read;
            double datum_offset_read = -999.0;
            // NOTE: Using DbTransferType::ReadRaw rather than
            // DbTransferType::Read here, because the latter would subtract
            // the offset from the interval and thus make the comparison invalid
            mdb_result = transaction->fpuDbTransferInterval(DbTransferType::ReadRaw,
                                                            interval_type,
                                                            serial_number_str.c_str(),
                                                            interval_read,
                                                            datum_offset_read);
            if (mdb_result == MDB_SUCCESS)
            {
                // Compare read and written, and indicate if any difference
                if ((interval_read != interval_write) ||
                    (datum_offset_read != datum_offset_write))
                {
                    mdb_result = MDB_VERIFY_FAILED;
                }
            }
        }
    }
    
    return mdb_result;
}

//------------------------------------------------------------------------------
MdbResult ProtectionDBTester::testFpuCountersTransfer(ProtectionDB &protectiondb)
{
    // Tests writing and reading back the counters for a single FPU
    
    MdbResult mdb_result = MDB_PANIC;
    
    auto transaction = protectiondb.createTransaction(mdb_result);
    if (transaction)
    {
        FpuCounters fpu_counters_write;
        for (int i = 0; i < (int)FpuCounterId::NumCounters; i++)
        {
            fpu_counters_write.setCount((FpuCounterId)i, i * 10);
        }

        std::string serial_number_str = getRandomFpuTestSerialNumber();

        // Write counters
        mdb_result = transaction->fpuDbTransferCounters(DbTransferType::Write,
                                                        serial_number_str.c_str(),
                                                        fpu_counters_write);
        if (mdb_result == MDB_SUCCESS)
        {
            // Read back counters
            FpuCounters fpu_counters_read;
            mdb_result = transaction->fpuDbTransferCounters(DbTransferType::Read,
                                                            serial_number_str.c_str(),
                                                            fpu_counters_read);
            if (mdb_result == MDB_SUCCESS)
            {
                // Compare read and written, and indicate if any difference
                for (int i = 0; i < (int)FpuCounterId::NumCounters; i++)
                {
                    if (fpu_counters_read.getCount((FpuCounterId)i) !=
                        fpu_counters_write.getCount((FpuCounterId)i))
                    {
                        mdb_result = MDB_VERIFY_FAILED;
                        break;
                    }
                }
            }
        }
    }
    
    return mdb_result;
}

//------------------------------------------------------------------------------
MdbResult ProtectionDBTester::testFpuWaveformTransfer(ProtectionDB &protectiondb)
{
    // Tests writing and reading back of an FPU waveform
    
    MdbResult mdb_result = MDB_PANIC;
    
    auto transaction = protectiondb.createTransaction(mdb_result);
    if (transaction)
    {
        std::string serial_number_str = getRandomFpuTestSerialNumber();

        // Write waveform
        t_waveform_steps waveform_write = 
            {{1, -2}, {-3, 4}, {50, 60}, {7, 8}, {9, 10}};
        mdb_result = transaction->fpuDbTransferWaveform(DbTransferType::Write,
                                                    serial_number_str.c_str(),
                                                    waveform_write);
        if (mdb_result == MDB_SUCCESS)
        {
            // Read back waveform
            t_waveform_steps waveform_read;
            mdb_result = transaction->fpuDbTransferWaveform(DbTransferType::Read,
                                                    serial_number_str.c_str(),
                                                    waveform_read);
            if (mdb_result == MDB_SUCCESS)
            {
                if (waveform_read.size() == waveform_write.size())
                {
                    for (size_t i = 0; i < waveform_read.size(); i++)
                    {
                        if ((waveform_read[i].alpha_steps !=
                             waveform_write[i].alpha_steps) ||
                            (waveform_read[i].beta_steps != 
                             waveform_write[i].beta_steps))
                        {
                            mdb_result = MDB_VERIFY_FAILED;
                            break;
                        }
                    }
                }
                else
                {
                    mdb_result = MDB_VERIFY_FAILED;
                }
            }
        }
    }

    return mdb_result;
}

//------------------------------------------------------------------------------
MdbResult ProtectionDBTester::testFpuInt64ValTransfer(ProtectionDB &protectiondb)
{
    MdbResult mdb_result = MDB_PANIC;

    auto transaction = protectiondb.createTransaction(mdb_result);
    if (transaction)
    {
        std::string serial_number_str = getRandomFpuTestSerialNumber();

        // Write int64_t value
        int64_t int64_val_write = 0x123456789abcdef0;
        mdb_result = transaction->fpuDbTransferInt64Val(DbTransferType::Write,
                                            FpuDbIntValType::BetaRetries_ACW,
                                            serial_number_str.c_str(),
                                            int64_val_write);
        if (mdb_result == MDB_SUCCESS)
        {
            // Read back int64_t value
            int64_t int64_val_read;
            mdb_result = transaction->fpuDbTransferInt64Val(DbTransferType::Read,
                                            FpuDbIntValType::BetaRetries_ACW,
                                            serial_number_str.c_str(),
                                            int64_val_read);
            if (mdb_result == MDB_SUCCESS)
            {
                if (int64_val_read != int64_val_write)
                {
                    mdb_result = MDB_VERIFY_FAILED;
                }
            }
        }
    }

    return mdb_result;
}

//------------------------------------------------------------------------------
MdbResult ProtectionDBTester::testFpuWfReversedFlagTransfer(ProtectionDB &protectiondb)
{
    MdbResult mdb_result = MDB_PANIC;

    auto transaction = protectiondb.createTransaction(mdb_result);
    if (transaction)
    {
        std::string serial_number_str = getRandomFpuTestSerialNumber();

        // Write wf_reversed bool value
        bool wf_reversed_write = true;
        mdb_result = transaction->fpuDbTransferWfReversedFlag(
                                                    DbTransferType::Write,
                                                    serial_number_str.c_str(),
                                                    wf_reversed_write);
        if (mdb_result == MDB_SUCCESS)
        {
            // Read back wf_reversed value
            bool wf_reversed_read = false;
            mdb_result = transaction->fpuDbTransferWfReversedFlag(
                                                    DbTransferType::Read,
                                                    serial_number_str.c_str(),
                                                    wf_reversed_read);
            if (mdb_result == MDB_SUCCESS)
            {
                if (wf_reversed_read != wf_reversed_write)
                {
                    mdb_result = MDB_VERIFY_FAILED;
                }
            }
        }
    }
    
    return mdb_result;
}

//------------------------------------------------------------------------------
MdbResult ProtectionDBTester::testFullFpuDataTransfer(ProtectionDB &protectiondb)
{
    MdbResult mdb_result = MDB_PANIC;

    auto transaction = protectiondb.createTransaction(mdb_result);
    if (transaction)
    {
        std::string serial_number_str = getRandomFpuTestSerialNumber();

        // Create a full fpu data structure, and populate it with test data
        FpuDbData fpu_db_data_write;
        fillFpuDbDataStructWithTestVals(fpu_db_data_write);

        // Write full fpu data structure
        mdb_result = transaction->fpuDbTransferFpu(DbTransferType::Write,
                                                   serial_number_str.c_str(),
                                                   fpu_db_data_write);
        if (mdb_result == MDB_SUCCESS)
        {
            // Read back full fpu data structure
            // NOTE: Using DbTransferType::ReadRaw rather than
            // DbTransferType::Read here, so that the RAW intervals (apos/bpos/
            // alimits/blimits) are read, rather than the offset-modified
            // ones, so that should read back the same as what was written above
            FpuDbData fpu_db_data_read;
            mdb_result = transaction->fpuDbTransferFpu(DbTransferType::ReadRaw,
                                                       serial_number_str.c_str(),
                                                       fpu_db_data_read);
            if (mdb_result == MDB_SUCCESS)
            {
                if (fpu_db_data_read != fpu_db_data_write)
                {
                    mdb_result = MDB_VERIFY_FAILED;
                }
            }
        }
    }

    return mdb_result;
}

//------------------------------------------------------------------------------
MdbResult ProtectionDBTester::testFpuSingleItemWriteRead(ProtectionDB &protectiondb)
{
    // Tests writing of a single item and reading it back, all in one transaction
    
    MdbResult mdb_result = MDB_PANIC;
    
    auto transaction = protectiondb.createTransaction(mdb_result);
    if (transaction)
    {
        // Write item
        std::string serial_number_str = getRandomFpuTestSerialNumber();
        char subkey[] = "TestSubkey";
        char data_str[] = "0123456789";
        mdb_result = transaction->fpuDbWriteItem(serial_number_str.c_str(),
                                                 subkey, (void *)data_str,
                                                 strlen(data_str));

        // Get item buffer
        void *item_data_ptr = nullptr;
        int item_num_bytes = 0;
        if (mdb_result == MDB_SUCCESS)
        {
            mdb_result = 
                transaction->fpuDbGetItemDataPtrAndSize(serial_number_str.c_str(),
                                                        subkey, &item_data_ptr,
                                                        item_num_bytes);
        }
        
        // Verify that item read back matches item written
        if (mdb_result == MDB_SUCCESS)
        {
            if ((item_num_bytes != (int)strlen(data_str)) ||
                (memcmp(data_str, item_data_ptr, strlen(data_str)) != 0))
            {
                mdb_result = MDB_VERIFY_FAILED;
            }
        }
    }
    
    return mdb_result;
}

//------------------------------------------------------------------------------
MdbResult ProtectionDBTester::testFpuMultipleItemWriteReads(ProtectionDB &protectiondb)
{
    // Tests writing of multiple items in a first transaction, and reading them
    // back in a second transaction

    MdbResult mdb_result = MDB_PANIC;
    const int num_iterations = 100;
    std::string serial_number_str = getRandomFpuTestSerialNumber();
    uint64_t test_multiplier = 0x123456789abcdef0L;
    
    // N.B. The transactions in the following code are each in their own scope
    // so that writes should be automatically committed when transaction goes out
    // of scope and is destroyed
    
    {
        auto transaction = protectiondb.createTransaction(mdb_result);
        if (transaction)
        {
            mdb_result = MDB_SUCCESS;
            for (int i = 0; i < num_iterations; i++)
            {
                char subkey_str[10];
                snprintf(subkey_str, sizeof(subkey_str), "%03d", i);
                uint64_t test_val = ((uint64_t)i) * test_multiplier;
                mdb_result = transaction->fpuDbWriteItem(serial_number_str.c_str(),
                                                 subkey_str, (void *)&test_val,
                                                 sizeof(test_val));
                if (mdb_result != MDB_SUCCESS)   
                {
                    // Error
                    break;
                }
            }
        }
    }

    if (mdb_result == MDB_SUCCESS)
    {
        auto transaction = protectiondb.createTransaction(mdb_result);
        if (transaction)
        {
            mdb_result = MDB_SUCCESS;
            for (int i = 0; i < num_iterations; i++)
            {
                char subkey_str[10];
                snprintf(subkey_str, sizeof(subkey_str), "%03d", i);
                uint64_t test_val = ((uint64_t)i) * test_multiplier;
                void *item_data_ptr = nullptr;
                int item_num_bytes = 0;
                mdb_result = transaction->fpuDbGetItemDataPtrAndSize(
                                                    serial_number_str.c_str(),
                                                    subkey_str, &item_data_ptr,
                                                    item_num_bytes);
                if (mdb_result == MDB_SUCCESS)    
                {
                    if ((item_num_bytes != sizeof(test_val)) ||
                        (memcmp(item_data_ptr, (void *)&test_val,
                                sizeof(test_val)) != 0))
                    {
                        mdb_result = MDB_VERIFY_FAILED;
                        break;
                    }
                }
                else
                {
                    break;
                }
            }
        }
    }
    
    return mdb_result;
}

//------------------------------------------------------------------------------
bool ProtectionDBTester::testFpuDbDataClass()
{
    bool result_ok = false;
    FpuDbData fpu_db_data_1;

    fillFpuDbDataStructWithTestVals(fpu_db_data_1);

    // Test the "==" and "!=" operator overloads for when the objects are the
    // same
    FpuDbData fpu_db_data_2 = fpu_db_data_1;
    if (fpu_db_data_2 == fpu_db_data_1)
    {
        result_ok = true;
        if (result_ok)
        {
            if (fpu_db_data_2 != fpu_db_data_1)
            {
                result_ok = false;
            }
        }
    }
    else
    {
        result_ok = false;
    }
    
    if (result_ok)
    {
        FpuDbData fpu_db_data_3 = fpu_db_data_1;
        
        // Test differences with a few different data types
        fpu_db_data_3.apos = Interval(123.0, 456.0);
        if (!(fpu_db_data_3 != fpu_db_data_1))
        {
            result_ok = false;
        }
        
        if (result_ok)
        {
            fpu_db_data_3 = fpu_db_data_1;
            fpu_db_data_3.counters.setCount(FpuCounterId::collisions, 123456);
            if (fpu_db_data_3 == fpu_db_data_1)
            {
                result_ok = false;
            }
        }
        
        if (result_ok)
        {
            fpu_db_data_3 = fpu_db_data_1;
            fpu_db_data_3.last_waveform[0].beta_steps = 999;
            if (fpu_db_data_3 == fpu_db_data_1)
            {
                result_ok = false;
            }
        }
    }
    
    return result_ok;
}

//------------------------------------------------------------------------------
void ProtectionDBTester::fillFpuDbDataStructWithTestVals(FpuDbData &fpu_db_data)
{
    fpu_db_data.snum_used_flag = SNUM_USED_CHECK_VAL;
    fpu_db_data.apos = Interval(1.0, 2.0);
    fpu_db_data.bpos = Interval(3.0, 4.0);
    fpu_db_data.wf_reversed = true;
    fpu_db_data.alimits = Interval(5.0, 6.0);
    fpu_db_data.blimits = Interval(7.0, 8.0);
    fpu_db_data.maxaretries = 9;
    fpu_db_data.aretries_cw = 10;
    fpu_db_data.aretries_acw = 11;
    fpu_db_data.maxbretries = 12;
    fpu_db_data.bretries_cw = 13;
    fpu_db_data.bretries_acw = 14;
    for (int i = 0; i < (int)FpuCounterId::NumCounters; i++)
    {
        fpu_db_data.counters.setCount((FpuCounterId)i, i * 100);
    }
    const int num_waveform_steps = 10;
    fpu_db_data.last_waveform.resize(num_waveform_steps);
    for (int i = 0; i < num_waveform_steps; i++)
    {
        fpu_db_data.last_waveform[i] = { (int16_t)(i + 100), (int16_t)(i + 200) };
    }
}

//------------------------------------------------------------------------------
std::string ProtectionDBTester::getRandomFpuTestSerialNumber()
{
    // Provides serial number strings of the form "TstNNN", with the NNN
    // characters being a random leading-zero-padded number between 000 and
    // 999.
    // N.B. The maximum allowed serial number length is given by
    // ethercanif::DIGITS_SERIAL_NUMBER

    // Seed the random number generator exactly once
    static bool seeded = false;
    if (!seeded)
    {
        time_t t;
        srand((unsigned)time(&t));
        seeded = true;
    }
   
    int number = rand() % 1000; // 0-999
    
    char serial_number_c_str[20];
    snprintf(serial_number_c_str, sizeof(serial_number_c_str), "Tst%03d", number);

    return std::string(serial_number_c_str);
}

//------------------------------------------------------------------------------
void ProtectionDBTester::testGetSerialNumFromKeyVal()
{
    // Ad-hoc testing of ProtectionDbTxn::fpuDbGetSerialNumFromKeyVal() -
    // single-step and check the results
    // NOTE: The following test strings assume that fpudb_keystr_separator_char
    // is '#'
    static const char *test_key_strs[] =
    {
        "",
        "PT01#qwerty",
        "abcdef",
        "abcdef#",
        "abcdef#vvv",
        "abcdefghijklmnopqrstuvwxyz",
        "ABCDEFGHIJ#abc",
        "QWERTYUIOPZ#def",
        "AB23#",
        "CD45"
    };
    MDB_val key_val;
    std::string serial_number;

    bool converted_ok = false;
    for (size_t i = 0; i < sizeof(test_key_strs) / sizeof(test_key_strs[0]); i++)
    {
        key_val.mv_data = (void *)test_key_strs[i];
        key_val.mv_size = strlen((char *)key_val.mv_data);
        converted_ok = ProtectionDbTxn::fpuDbGetSerialNumFromKeyVal(key_val,
                                                                    serial_number);
    }
}

//------------------------------------------------------------------------------
void ProtectionDBTester::testDbOpeningScenarios()
{
    // TODO: This function is WIP - will come back to once have better
    // ProtectionDB / ProtectionDBTxn return codes in place


    // Ad-hoc testing of ProtectionDB::open(), to check behaviour with a few of
    // the following scenarios:
    //   - Specified directory exists or doesn't exist
    //   - Database files already exist or don't exist
    //   - Sub-databases (e.g. fpudb) already exist or don't exist inside the
    //     database files
    // Note: Each protectiondb instance is tried in its own code scope, so that
    // it's automatically closed again once it goes out of scope

    volatile MdbResult mdb_result = MDB_PANIC;  // volatile so not optimised away

    {
        // Test for when directory, files and sub-databases all exist - N.B.
        // they need to all be present for this test to pass
        ProtectionDB protectiondb;
        const bool mockup = true;
        std::string dir_str = ProtectionDB::getDirFromLinuxEnv(mockup);
        mdb_result = protectiondb.open(dir_str);
    }

    {
        ProtectionDB protectiondb;
        std::string dir_str = "/moonsdata/shouldnt_exist";
        mdb_result = protectiondb.open(dir_str);
    }

}

//------------------------------------------------------------------------------

