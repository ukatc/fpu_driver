// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-08-27  TODO: Put comment here
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME ProtectionDBTester.C
//
// Provides test functionality for the protection database classes.
//
////////////////////////////////////////////////////////////////////////////////

#include <string.h>
#include "ProtectionDBTester.h"

//------------------------------------------------------------------------------
bool ProtectionDBTester::doLoopingTestsWithConsoleOutput()
{
    const int num_iterations = 1000;
    for (int i = 0; i < num_iterations; i++)
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
            break;
        }
    }
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
    //     numbers (see getNextFpuTestSerialNumber()) which it then uses to
    //     write to and read from the FPU database. If an item with the same
    //     serial number / subkey combination already exists in the database
    //     then it is overwritten
    
    std::string dir_str = "/moonsdata/fpudb_NEWFORMAT";
    bool result_ok = false;

    // TODO: All test functions in this file should go into ProtectionDBTester,
    // so the code in this bit should be consolidated with it once this is done
    ProtectionDBTester protection_db_tester;
    result_ok = protection_db_tester.testFpuDbDataClass();
    
    if (result_ok)
    {
        result_ok = testWithStayingOpen(dir_str);
    }
    
    if (result_ok)
    {
        result_ok = testWithClosingReopening(dir_str);
    }
    
    return result_ok;
}

//------------------------------------------------------------------------------
bool ProtectionDBTester::testWithStayingOpen(const std::string &dir_str)
{
    // Performs various ProtectionDB tests with the database being kept open
    // NOTE: An LMDB database must already exist in dir_str location

    ProtectionDB protectiondb;
    bool result_ok = false;
   
    if (protectiondb.open(dir_str))
    {
        result_ok = testFpuSingleItemWriteRead(protectiondb);
        
        if (result_ok)
        {
            result_ok = testFpuMultipleItemWriteReads(protectiondb);
        }

        if (result_ok)
        {
            result_ok = testFpuPositionTransfer(protectiondb);
        }
        
        if (result_ok)
        {
            result_ok = testFpuCountersTransfer(protectiondb);
        }
        
        if (result_ok)
        {
            result_ok = testFpuWaveformTransfer(protectiondb);
        }
        
        if (result_ok)
        {
            result_ok = testFpuInt64ValTransfer(protectiondb);
        }
        
        if (result_ok)
        {
            result_ok = testFpuWfReversedFlagTransfer(protectiondb);
        }
        
        if (result_ok)
        {
            result_ok = testFullFpuDataTransfer(protectiondb);
        }
        
        // Run the sync() function
        if (result_ok)
        {
            result_ok = protectiondb.sync();
        }
    }
    
    return result_ok;
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

    bool result_ok = false;
    
    {
        ProtectionDB protectiondb;
        if (protectiondb.open(dir_str))
        {
            result_ok = testFpuSingleItemWriteRead(protectiondb);
        }
        else
        {
            result_ok = false;
        }
    }
    
    if (result_ok)
    {
        ProtectionDB protectiondb;
        if (protectiondb.open(dir_str))
        {
            result_ok = testFpuMultipleItemWriteReads(protectiondb);
        }
        else
        {
            result_ok = false;
        }
    }

    if (result_ok)
    {
        ProtectionDB protectiondb;
        if (protectiondb.open(dir_str))
        {
            result_ok = testFpuPositionTransfer(protectiondb);
        }
        else
        {
            result_ok = false;
        }
    }
    
    if (result_ok)
    {
        ProtectionDB protectiondb;
        if (protectiondb.open(dir_str))
        {
            result_ok = testFpuCountersTransfer(protectiondb);
        }
        else
        {
            result_ok = false;
        }
    }
    
    if (result_ok)
    {
        ProtectionDB protectiondb;
        if (protectiondb.open(dir_str))
        {
            result_ok = testFpuWaveformTransfer(protectiondb);
        }
        else
        {
            result_ok = false;
        }
    }
    
    if (result_ok)
    {
        ProtectionDB protectiondb;
        if (protectiondb.open(dir_str))
        {
            result_ok = testFpuInt64ValTransfer(protectiondb);
        }
        else
        {
            result_ok = false;
        }
    }
    
    if (result_ok)
    {
        ProtectionDB protectiondb;
        if (protectiondb.open(dir_str))
        {
            result_ok = testFpuWfReversedFlagTransfer(protectiondb);
        }
        else
        {
            result_ok = false;
        }
    }
    
    if (result_ok)
    {
        ProtectionDB protectiondb;
        if (protectiondb.open(dir_str))
        {
            result_ok = testFullFpuDataTransfer(protectiondb);
        }
        else
        {
            result_ok = false;
        }
    }
    
    return result_ok;    
}

//------------------------------------------------------------------------------
bool ProtectionDBTester::testFpuPositionTransfer(ProtectionDB &protectiondb)
{
    // Tests writing and reading back an aggregate position value (interval +
    // datum offset) of an FPU
    
    bool result_ok = false;
    
    auto transaction = protectiondb.createTransaction();
    if (transaction)
    {
        FpuDbPositionType position_type = FpuDbPositionType::BetaLimit;
        Interval interval_write(1.2, 3.4);
        double datum_offset_write = 180.0;
        std::string serial_number_str = getNextFpuTestSerialNumber();

        // Write position value
        result_ok = transaction->fpuDbTransferPosition(DbTransferType::Write,
                                                       position_type,
                                                       serial_number_str.c_str(),
                                                       interval_write,
                                                       datum_offset_write);
        if (result_ok)
        {
            // Read back position value
            Interval interval_read;
            double datum_offset_read = -999.0;
            result_ok = transaction->fpuDbTransferPosition(DbTransferType::Read,
                                                           position_type,
                                                           serial_number_str.c_str(),
                                                           interval_read,
                                                           datum_offset_read);
            if (result_ok)
            {
                // Compare read and written, and indicate if any difference
                if ((interval_read != interval_write) ||
                    (datum_offset_read != datum_offset_write))
                {
                    result_ok = false;
                }
            }
        }
    }
    
    return result_ok;
}

//------------------------------------------------------------------------------
bool ProtectionDBTester::testFpuCountersTransfer(ProtectionDB &protectiondb)
{
    // Tests writing and reading back the counters for a single FPU
    
    bool result_ok = false;
    
    auto transaction = protectiondb.createTransaction();
    if (transaction)
    {
        FpuCounters fpu_counters_write;
        for (int i = 0; i < (int)FpuCounterId::NumCounters; i++)
        {
            fpu_counters_write.setCount((FpuCounterId)i, i * 10);
        }

        std::string serial_number_str = getNextFpuTestSerialNumber();

        // Write counters
        result_ok = transaction->fpuDbTransferCounters(DbTransferType::Write,
                                                       serial_number_str.c_str(),
                                                       fpu_counters_write);
        if (result_ok)
        {
            // Read back counters
            FpuCounters fpu_counters_read;
            result_ok = transaction->fpuDbTransferCounters(DbTransferType::Read,
                                                           serial_number_str.c_str(),
                                                           fpu_counters_read);
            if (result_ok)
            {
                // Compare read and written, and indicate if any difference
                for (int i = 0; i < (int)FpuCounterId::NumCounters; i++)
                {
                    if (fpu_counters_read.getCount((FpuCounterId)i) !=
                        fpu_counters_write.getCount((FpuCounterId)i))
                    {
                        result_ok = false;
                        break;
                    }
                }
            }
        }
    }
    
    return result_ok;
}

//------------------------------------------------------------------------------
bool ProtectionDBTester::testFpuWaveformTransfer(ProtectionDB &protectiondb)
{
    // Tests writing and reading back of an FPU waveform
    
    bool result_ok = false;
    
    auto transaction = protectiondb.createTransaction();
    if (transaction)
    {
        std::string serial_number_str = getNextFpuTestSerialNumber();

        // Write waveform
        t_waveform_steps waveform_write = 
            {{1, -2}, {-3, 4}, {50, 60}, {7, 8}, {9, 10}};
        result_ok = transaction->fpuDbTransferWaveform(DbTransferType::Write,
                                                    serial_number_str.c_str(),
                                                    waveform_write);
        if (result_ok)
        {
            // Read back waveform
            t_waveform_steps waveform_read;
            result_ok = transaction->fpuDbTransferWaveform(DbTransferType::Read,
                                                    serial_number_str.c_str(),
                                                    waveform_read);
            if (result_ok)
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
                            result_ok = false;
                            break;
                        }
                    }
                }
                else
                {
                    result_ok = false;
                }
            }
        }
    }

    return result_ok;
}

//------------------------------------------------------------------------------
bool ProtectionDBTester::testFpuInt64ValTransfer(ProtectionDB &protectiondb)
{
    bool result_ok = false;

    auto transaction = protectiondb.createTransaction();
    if (transaction)
    {
        std::string serial_number_str = getNextFpuTestSerialNumber();

        // Write int64_t value
        int64_t int64_val_write = 0x123456789abcdef0;
        result_ok = transaction->fpuDbTransferInt64Val(DbTransferType::Write,
                                            FpuDbIntValType::BetaRetries_ACW,
                                            serial_number_str.c_str(),
                                            int64_val_write);
        if (result_ok)
        {
            // Read back int64_t value
            int64_t int64_val_read;
            result_ok = transaction->fpuDbTransferInt64Val(DbTransferType::Read,
                                            FpuDbIntValType::BetaRetries_ACW,
                                            serial_number_str.c_str(),
                                            int64_val_read);
            if (result_ok)
            {
                if (int64_val_read != int64_val_write)
                {
                    result_ok = false;
                }
            }
        }
    }

    return result_ok;
}

//------------------------------------------------------------------------------
bool ProtectionDBTester::testFpuWfReversedFlagTransfer(ProtectionDB &protectiondb)
{
    bool result_ok = false;

    auto transaction = protectiondb.createTransaction();
    if (transaction)
    {
        std::string serial_number_str = getNextFpuTestSerialNumber();

        // Write wf_reversed bool value
        bool wf_reversed_write = true;
        result_ok = transaction->fpuDbTransferWfReversedFlag(
                                                    DbTransferType::Write,
                                                    serial_number_str.c_str(),
                                                    wf_reversed_write);
        if (result_ok)
        {
            // Read back wf_reversed value
            bool wf_reversed_read = false;
            result_ok = transaction->fpuDbTransferWfReversedFlag(
                                                    DbTransferType::Read,
                                                    serial_number_str.c_str(),
                                                    wf_reversed_read);
            if (result_ok)
            {
                if (wf_reversed_read != wf_reversed_write)
                {
                    result_ok = false;
                }
            }
        }
    }
    
    return result_ok;
}

//------------------------------------------------------------------------------
bool ProtectionDBTester::testFullFpuDataTransfer(ProtectionDB &protectiondb)
{
    bool result_ok = false;

    auto transaction = protectiondb.createTransaction();
    if (transaction)
    {
        std::string serial_number_str = getNextFpuTestSerialNumber();

        // Create a full fpu data structure, and populate it with test data
        FpuDbData fpu_db_data_write;
        fillFpuDbDataStructWithTestVals(fpu_db_data_write);

        // Write full fpu data structure
        result_ok = transaction->fpuDbTransferFpu(DbTransferType::Write,
                                                  serial_number_str.c_str(),
                                                  fpu_db_data_write);
        if (result_ok)
        {
            // Read back full fpu data structure
            FpuDbData fpu_db_data_read;
            result_ok = transaction->fpuDbTransferFpu(DbTransferType::Read,
                                                      serial_number_str.c_str(),
                                                      fpu_db_data_read);
            if (result_ok)
            {
                if (fpu_db_data_read != fpu_db_data_write)
                {
                    result_ok = false;
                }
            }
        }
    }

    return result_ok;
}

//------------------------------------------------------------------------------
bool ProtectionDBTester::testFpuSingleItemWriteRead(ProtectionDB &protectiondb)
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
        result_ok = transaction->fpuDbWriteItem(serial_number_str.c_str(),
                                                subkey, (void *)data_str,
                                                strlen(data_str));

        // Get item buffer
        void *item_data_ptr = nullptr;
        int item_num_bytes = 0;
        if (result_ok)
        {
            result_ok = 
                transaction->fpuDbGetItemDataPtrAndSize(serial_number_str.c_str(),
                                                        subkey, &item_data_ptr,
                                                        item_num_bytes);
        }
        
        // Verify that item read back matches item written
        if (result_ok)
        {
            if ((item_num_bytes != strlen(data_str)) ||
                (memcmp(data_str, item_data_ptr, strlen(data_str)) != 0))
            {
                result_ok = false;
            }
        }
    }
    
    return result_ok;
}

//------------------------------------------------------------------------------
bool ProtectionDBTester::testFpuMultipleItemWriteReads(ProtectionDB &protectiondb)
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
                if (!transaction->fpuDbWriteItem(serial_number_str.c_str(),
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
                void *item_data_ptr = nullptr;
                int item_num_bytes = 0;
                if (transaction->fpuDbGetItemDataPtrAndSize(serial_number_str.c_str(),
                                                            subkey_str,
                                                            &item_data_ptr,
                                                            item_num_bytes))
                {
                    if ((item_num_bytes != sizeof(test_val)) ||
                        (memcmp(item_data_ptr, (void *)&test_val,
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
    
    // TODO: Return true or false
    return result_ok;
}

//------------------------------------------------------------------------------
void ProtectionDBTester::fillFpuDbDataStructWithTestVals(FpuDbData &fpu_db_data)
{
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
std::string ProtectionDBTester::getNextFpuTestSerialNumber()
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

