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

//..............................................................................

static bool protectionDB_TestWithStayingOpen(const std::string &dir_str);
static bool protectionDB_TestWithClosingReopening(const std::string &dir_str);
static bool protectionDB_TestSingleFpuCountersTransfer(ProtectionDB &protectiondb);
static bool protectionDB_TestSinglePositionTransfer(ProtectionDB &protectiondb);
static bool protectionDB_TestSingleItemWriteRead(ProtectionDB &protectiondb);
static bool protectionDB_TestMultipleItemWriteReads(ProtectionDB &protectiondb);
static std::string getNextFpuTestSerialNumber();

//------------------------------------------------------------------------------
void testFpuCounters()
{
    FpuCounters fpu_counters;
    
    for (int id = 0; id < (int)FpuCounterId::NumCounters; id++)
    {
        fpu_counters.setCount((FpuCounterId)id, ((int)id) * 10);
    }
    
    fpu_counters.zeroAll();
    
    fpu_counters.addToCount(FpuCounterId::alpha_starts, 345);
    FpuCounterInt astarts = fpu_counters.getCount(FpuCounterId::alpha_starts);
    
    fpu_counters.addToCount(FpuCounterId::datum_count, 22);
    fpu_counters.addToCount(FpuCounterId::datum_count, 0x1234567890);
    FpuCounterInt datums = fpu_counters.getCount(FpuCounterId::datum_count);
    
    FpuCounterInt dummy_array[(int)FpuCounterId::NumCounters];
    for (int i = 0; i < (int)FpuCounterId::NumCounters; i++)
    {
        dummy_array[i] = i * 100;
    }
    fpu_counters.zeroAll();
    fpu_counters.populateFromRawBytes(dummy_array);
    
    for (int i = 0; i < (int)FpuCounterId::NumCounters; i++)
    {
        dummy_array[i] = 0;
    }
    void *raw_bytes_ptr = fpu_counters.getRawBytesPtr();
    int num_raw_bytes = fpu_counters.getNumRawBytes();
    memcpy(dummy_array, raw_bytes_ptr, num_raw_bytes);
}

//------------------------------------------------------------------------------
bool protectionDB_Test()
{
    // Performs a suite of protection database tests - reading and writing
    // items within individual or multiple transactions, also with some 
    // database closing/re-opening
    // IMPORTANT NOTES:
    //   - An LMDB database must already exist in dir_str location (see below)
    //   - This test functionality currently generates random FPU serial
    //     numbers (see getNextFpuTestSerialNumber()) which it then uses to
    //     write to and read from the FPU database - however, if one of these
    //     FPU serial numbers already exists in the database then a test might
    //     occasionally fail
    //     *** TODO: *** Fix this eventually
    
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
        result_ok = protectionDB_TestSingleItemWriteRead(protectiondb);
        
        if (result_ok)
        {
            result_ok = protectionDB_TestMultipleItemWriteReads(protectiondb);
        }

        if (result_ok)
        {
            result_ok = protectionDB_TestSingleFpuCountersTransfer(protectiondb);
        }
        
        if (result_ok)
        {
            result_ok = protectionDB_TestSinglePositionTransfer(protectiondb);
        }
    }
    
    return result_ok;
}

//------------------------------------------------------------------------------
static bool protectionDB_TestWithClosingReopening(const std::string &dir_str)
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
            result_ok = protectionDB_TestSingleItemWriteRead(protectiondb);
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
            result_ok = protectionDB_TestMultipleItemWriteReads(protectiondb);
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
            result_ok = protectionDB_TestSingleFpuCountersTransfer(protectiondb);
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
            result_ok = protectionDB_TestSinglePositionTransfer(protectiondb);
        }
        else
        {
            result_ok =  false;
        }
    }
    
    return result_ok;    
}

//------------------------------------------------------------------------------
static bool protectionDB_TestSingleFpuCountersTransfer(ProtectionDB &protectiondb)
{
    // Tests writing and reading back the counters for a single FPU
    
    bool result_ok = false;
    
    auto transaction = protectiondb.createTransaction();
    if (transaction)
    {
        FpuCounters fpu_counters_to_write;
        for (int i = 0; i < (int)FpuCounterId::NumCounters; i++)
        {
            fpu_counters_to_write.setCount((FpuCounterId)i, i * 10);
        }

        std::string serial_number_str = getNextFpuTestSerialNumber();

        // Write counters
        result_ok = transaction->fpuDbTransferCounters(DbTransferType::Write,
                                                       serial_number_str.c_str(),
                                                       fpu_counters_to_write);
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
                        fpu_counters_to_write.getCount((FpuCounterId)i))
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
static bool protectionDB_TestSinglePositionTransfer(ProtectionDB &protectiondb)
{
    // Tests writing and reading back an aggregate position value (interval +
    // datum offset) of an FPU
    
    bool result_ok = false;
    
    auto transaction = protectiondb.createTransaction();
    if (transaction)
    {
        FpuDbPositionType position_type = FpuDbPositionType::BetaLimit;
        Interval interval_to_write(1.2, 3.4);
        double datum_offset_to_write = 180.0;
        std::string serial_number_str = getNextFpuTestSerialNumber();

        // Write position value
        result_ok = transaction->fpuDbTransferPosition(DbTransferType::Write,
                                                       position_type,
                                                       serial_number_str.c_str(),
                                                       interval_to_write,
                                                       datum_offset_to_write);
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
                if ((interval_read != interval_to_write) ||
                    (datum_offset_read != datum_offset_to_write))
                {
                    result_ok = false;
                }
            }
        }
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

