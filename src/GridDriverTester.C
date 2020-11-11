// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-08-05  TODO: Put comment here
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME GridDriverTester.C
//
// Provides test functionality for the UnprotectedGridDriver and GridDriver
// classes.
//
////////////////////////////////////////////////////////////////////////////////

#include <vector>
#include <string>
#include <cstring>
#include "GridDriverTester.h"
#include "E_GridState.h"
#include "InterfaceState.h"
#include "ProtectionDBTester.h"

#define TESTING_MAX_NUM_FPUS    (5)

// NOTE: Can change TESTING_NUM_FPUS as required, up to TESTING_MAX_NUM_FPUS -
// and set mock gateway (if used) to the same value when invoking it
#define TESTING_NUM_FPUS        (3)
#if (TESTING_NUM_FPUS > TESTING_MAX_NUM_FPUS)
#error "BUILD ERROR: TESTING_NUM_FPUS > TESTING_MAX_NUM_FPUS"
#endif

namespace mpifps
{

//------------------------------------------------------------------------------
void GridDriverTester::doGridDriverUnitTests()
{
    // Performs ad-hoc unit tests on a GridDriver instance
    static const int num_fpus = 10;
    GridDriver gd(num_fpus);

    //..........................................................................
    // Test GridDriver::getDuplicateSerialNumbers()
    const char *test_snumbers[num_fpus] =   // NOTE: Must be 6 chars or less
    {                                       // (LEN_SERIAL_NUMBER - 1)
        "ab123",
        "ab12",
        "ab1234",
        "ac1234",
        "ab123",    // Duplicate
        "xy543",
        "xy5",
        "xy543",    // Duplicate
        "ac1234",   // Duplicate
        "qwerty"
    };
    t_grid_state grid_state;
    for (int fpu_id = 0; fpu_id < num_fpus; fpu_id++)
    {
        // N.B. Using safer strncpy() (rather than strcpy()))
        strncpy(grid_state.FPU_state[fpu_id].serial_number,
                test_snumbers[fpu_id], LEN_SERIAL_NUMBER);
        // Add guaranteed null-terminator at end of buffer
        grid_state.FPU_state[fpu_id].serial_number[LEN_SERIAL_NUMBER - 1] = '\0';
    }

#ifdef ENABLE_PROTECTION_CODE
    std::vector<std::string> duplicate_snumbers;
    gd.getDuplicateSerialNumbers(grid_state, duplicate_snumbers);
#endif
    
    //..........................................................................
    
    int dummy = 123;
    
}

//------------------------------------------------------------------------------
void GridDriverTester::doUnprotectedGridDriverFunctionalTesting()
{
    E_EtherCANErrCode ecan_result;
    
    UnprotectedGridDriver ugd(TESTING_NUM_FPUS);

    ecan_result = ugd.initialize();

    if (ecan_result == DE_OK)
    {
        const bool soft_protection = false;
        testInitialisedGridDriver(ugd, soft_protection);
    }
}

//------------------------------------------------------------------------------
void GridDriverTester::doGridDriverFunctionalTesting()
{
    // Performs a full grid driver functional test sequence
    
    E_EtherCANErrCode ecan_result;

    GridDriver gd(TESTING_NUM_FPUS);

    ecan_result = gd.initialize();

    if (ecan_result == DE_OK)
    {
        const bool mockup = true;
        ecan_result = gd.initProtection(mockup);
    }

    if (ecan_result == DE_OK)
    {
        const bool soft_protection = true;
        testInitialisedGridDriver(gd, soft_protection);
    }
}

//------------------------------------------------------------------------------
void GridDriverTester::testInitialisedGridDriver(UnprotectedGridDriver &gd,
                                                 bool soft_protection)
{
    // Performs basic functional testing of a pre-initialised 
    // UnprotectedGridDriver or GridDriver object, for up to 5 FPUs. Notes:
    //   - initialize(), initProtection() (for GridDriver) and any other
    //     initialisations must have been successfully performed for grid_driver
    //     before calling this function
    //   - Can be run using the mock gateway simulator - need to first set this
    //     running in a Bash Python shell in .../test/HardwareSimulation,
    //     using:
    //         python mock_gateway.py -N 5    (N.B. Adjust for number of FPUs)
    //   - In a debugger, can single-step and breakpoint this code and look at
    //     the return values to see what's going on, and also look at the 
    //     mock gateway console output

    E_EtherCANErrCode ecan_result;
    E_GridState grid_state_result;
    t_grid_state grid_state;

    t_fpuset fpuset;
    createFpuSetFlags(TESTING_NUM_FPUS, fpuset);

    //..........................................................................
    // Test connect()
    // TODO: t_gateway_address::ip is only a pointer - dangerous? Change this
    // eventually? (e.g. to a std::string?)
    ecan_result = gd.connect(1, &gateway_address);   

    //..........................................................................
    // Test getGridState()
    grid_state_result = gd.getGridState(grid_state);
    
    //..........................................................................
    // Test readSerialNumbers()
    ecan_result = gd.readSerialNumbers(grid_state, fpuset);
    
    //..........................................................................
    // Test pingFPUs()
    ecan_result = gd.pingFPUs(grid_state, fpuset);
    
    //..........................................................................
    // Test getGridState() again
    grid_state_result = gd.getGridState(grid_state);

    //..........................................................................
    // Test pingFPUs() again
    ecan_result = gd.pingFPUs(grid_state, fpuset);

    //..........................................................................
    // Test resetFPUs()
    ecan_result = gd.resetFPUs(grid_state, fpuset);
    
    //..........................................................................
    // Test findDatum()
    t_datum_search_flags search_modes;
    for (int fpu_id = 0; fpu_id < TESTING_NUM_FPUS; fpu_id++)
    {
        search_modes[fpu_id] = SEARCH_CLOCKWISE;
    }
    const bool count_protection = false;
    const bool support_uninitialized_auto = false;

    grid_state_result = gd.getGridState(grid_state);

    // TODO: Use DATUM_TIMEOUT_DISABLE instead of DATUM_TIMEOUT_ENABLE below?
    // (sometimes times out if long-duration findDatum())
    ecan_result = gd.findDatum(grid_state, search_modes, DASEL_BOTH, fpuset,
                               soft_protection, count_protection,
                               support_uninitialized_auto,
                               DATUM_TIMEOUT_ENABLE);

    //..........................................................................
    // Test configMotion() and wavetable_was_received()
    const bool allow_uninitialized = true;
    const int ruleset_version = DEFAULT_WAVEFORM_RULESET_VERSION;
    const bool warn_unsafe = true;
    const int verbosity = 3;

    t_wtable wavetable;
    static const t_waveform test_waveforms[TESTING_MAX_NUM_FPUS] =
    {
        {0, { { 0,  -1}, { 2,  -3}, { 4,  -5} } },
        {1, { { 6,  -7}, { 8,  -9}, {10, -11} } },
        {2, { {12, -13}, {14, -15}, {16, -17} } },
        {3, { {18, -19}, {20, -21}, {22, -23} } },
        {4, { {24, -25}, {26, -27}, {28, -29} } }
    };
    for (int i = 0; i < TESTING_NUM_FPUS; i++)
    {
        wavetable.push_back(test_waveforms[i]);
    }

    // Ad-hoc partial test of wavetable_was_received() (N.B. private function,
    // accessible because this test class is friend'ed from the grid driver
    // classes)
    t_fpu_state fpu_state;
    bool allow_unconfirmed = false;
    E_FPU_STATE target_state = FPST_READY_FORWARD;
    bool wtable_received_result = 
            gd.wavetable_was_received(wavetable, 3, fpu_state,
                                      allow_unconfirmed, target_state);
    UNUSED_ARG(wtable_received_result); // Suppress variable-not-used warning
    
    grid_state_result = gd.getGridState(grid_state);

    for (int i = 0; i < TESTING_NUM_FPUS; i++)
    {
        ecan_result = gd.enableMove(i, grid_state);
    }
    
    //........................
    // N.B. If required, can uncomment the
    // following code to test the wtable
    // pruning code inside configMotion()
    /*    
    if (TESTING_NUM_FPUS >= 2)
    {
        fpuset[1] = false;
    }
    if (TESTING_NUM_FPUS >= 4)
    {
        fpuset[3] = false;
    }
    */ 
    //........................
    
    ecan_result = gd.configMotion(wavetable, grid_state, fpuset,
                                  soft_protection, allow_uninitialized,
                                  ruleset_version, warn_unsafe, verbosity);

    //..........................................................................
    // Test executeMotion()
    bool sync_command = true;
    ecan_result = gd.executeMotion(grid_state, fpuset, sync_command);
    
    //..........................................................................

    ecan_result = gd.disconnect();

    //..........................................................................

    // Suppress warnings of variables not being used
    UNUSED_ARG(ecan_result);
    UNUSED_ARG(grid_state_result);
}

//------------------------------------------------------------------------------
bool GridDriverTester::writeGridFpusToFpuDb(int num_fpus, bool db_mockup)
{
    // *************************************************************************
    //     IMPORTANT: This test function will overwrite the FPU database's
    //       items for the first num_fpus x grid FPUs - use with caution
    // *************************************************************************
    //
    // Writes num_fpus FPU test data entries into the FPU database,
    // corresponding to the serial numbers of the first num_fpus FPUs available
    // on the grid.
    //
    // This function first uses an *UN*protectedGridDriver instance to read
    // the grid FPUs' serial numbers, because its connect() function will not
    // call the GridDriver::_post_connect_hook() function override (which would
    // fail if no FPU data was present yet). It then adds FPU data for these
    // serial numbers to the FPU database.
    //
    // NOTE: The gateway needs to be running before this function is called.
    // The expected protection database location is controlled by the db_mockup
    // flag - see ProtectionDB::getDirFromLinuxEnv().
    //
    // N.B. This function was written to support testing of
    // GridDriver::_post_connect_hook(), which expects the grid FPU entries to be
    // present in the FPU database.
    
    if ((num_fpus < 1) || (num_fpus >= MAX_NUM_POSITIONERS))
    {
        return false;
    }

    E_EtherCANErrCode ecan_result;
    t_grid_state grid_state;

    t_fpuset fpuset;
    createFpuSetFlags(num_fpus, fpuset);
    
    //..........................................................................
    // Get grid FPU serial numbers using an UnprotectedGridDriver instance
    UnprotectedGridDriver ugd(num_fpus);
    ecan_result = ugd.initialize();
    if (ecan_result == DE_OK)
    {
        ecan_result = ugd.connect(1, &gateway_address);
    }
    
    if (ecan_result == DE_OK)
    {
        ugd.getGridState(grid_state);
        ecan_result = ugd.readSerialNumbers(grid_state, fpuset);
    }
    
    std::vector<std::string> serial_numbers;
    if (ecan_result == DE_OK)
    {
        for (int i = 0; i < num_fpus; i++)
        {
            serial_numbers.push_back(grid_state.FPU_state[i].serial_number);
        }
    }

    ugd.disconnect();
    
    if (ecan_result != DE_OK)
    {
        return false;
    }

    //..........................................................................
    // Write grid FPU test items to FPU database
    FpuDbData fpu_db_data;
    ProtectionDBTester::fillFpuDbDataStructWithTestVals(fpu_db_data);

    bool result_ok = false;

    std::string dir_str = ProtectionDB::getDirFromLinuxEnv(db_mockup);
    if (!dir_str.empty())
    {
        ProtectionDB protectiondb;
        if (protectiondb.open(dir_str))
        {
            for (size_t i = 0; i < serial_numbers.size(); i++)
            {
                result_ok = false;
                auto txn = protectiondb.createTransaction();
                if (txn)
                {
                    if (txn->fpuDbTransferFpu(DbTransferType::Write,
                                              serial_numbers[i].c_str(),
                                              fpu_db_data))
                    {
                        result_ok = true;
                    }
                    else
                    {
                        break;
                    }
                }
            }
        }
    }

    //..........................................................................

    return result_ok;
}

//------------------------------------------------------------------------------
bool GridDriverTester::writeDummyFpuItemsToFpuDb(bool db_mockup,
                                                 const char *serial_number)
{
    // *************************************************************************
    //     IMPORTANT: This test function will overwrite the FPU database's
    //     serial_number FPU fields if they already exist - use with caution
    // *************************************************************************
    
    // Creates a set of dummy field items in the FPU database for an FPU
    // specified by serial_number.
    // NOTE: The database must not be currently opened by any other process
    // when calling this function.
    // The db_mockup flag controls the expected protection database location -
    // see ProtectionDB::getDirFromLinuxEnv().

    bool result_ok = false;
    std::string dir_str = ProtectionDB::getDirFromLinuxEnv(db_mockup);
    if (!dir_str.empty())
    {
        ProtectionDB protectiondb;
        if (protectiondb.open(dir_str))
        {
            auto txn = protectiondb.createTransaction();
            if (txn)
            {
                FpuDbData fpu_db_data;
                ProtectionDBTester::fillFpuDbDataStructWithTestVals(fpu_db_data);
                if (txn->fpuDbTransferFpu(DbTransferType::Write, serial_number,
                                          fpu_db_data))
                {
                    result_ok = true;
                }
            }
        }
    }
    return result_ok;
}

//------------------------------------------------------------------------------


} // namespace mpifps
