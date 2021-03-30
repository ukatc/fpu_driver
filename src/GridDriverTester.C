// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-08-05  Created.
// bwillemse 2021-03-26  Modified for new non-contiguous FPU IDs and CAN mapping.
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME GridDriverTester.C
//
// Provides test functionality for testing the UnprotectedGridDriver and
// GridDriver classes.
//
////////////////////////////////////////////////////////////////////////////////

#include <vector>
#include <string>
#include <cstring>
#include "GridDriverTester.h"
#include "E_GridState.h"
#include "InterfaceState.h"
#include "ErrorCodes.h"
#include "ProtectionDBTester.h"

#ifndef FLEXIBLE_CAN_MAPPING // NOT FLEXIBLE_CAN_MAPPING
#define TESTING_MAX_NUM_FPUS    (5)
#endif // NOT FLEXIBLE_CAN_MAPPING

namespace mpifps
{

#ifdef FLEXIBLE_CAN_MAPPING
static const std::string test_can_map_path("/blah/blah/blah.csv");
#endif // FLEXIBLE_CAN_MAPPING

//------------------------------------------------------------------------------
void GridDriverTester::doGridDriverUnitTests()
{
    // Performs ad-hoc unit tests on a GridDriver instance

#ifdef FLEXIBLE_CAN_MAPPING
    //..........................................................................
    // Test CAN map CSV file reading
    std::string csv_file_path("/home/bartw/BartsStuff/test_can_map.csv");
    GridCanMap grid_can_map;
    CanMapCsvFileResultInfo csv_file_result_info;
    gridDriverReadCanMapCsvFile(csv_file_path, grid_can_map,
                                csv_file_result_info);

    //..........................................................................
    // Test creating and initialising a GridDriver instance

    GridDriver gd;
    gd.initialize(test_can_map_path);

#else // NOT FLEXIBLE_CAN_MAPPING
    static const int num_fpus = 10;
    GridDriver gd(num_fpus);

    gd.initialize();

#endif // NOT FLEXIBLE_CAN_MAPPING

    //..........................................................................
    // Test UnprotectedGridDriver::wavetable_was_received()

    // Create a wavetable
    t_wtable wavetable;
    const std::vector<t_waveform> test_waveforms =
    {
        {0, getWaveform(GeneratedWaveform::Steps_10_10) },
        {1, getWaveform(GeneratedWaveform::Steps_20_20) },
        {2, getWaveform(GeneratedWaveform::Steps_90_90) },
        {3, getWaveform(GeneratedWaveform::Steps_Minus89_Minus89) },
        {4, getWaveform(GeneratedWaveform::Steps_10_10) }
    };
    for (size_t i = 0; i < test_waveforms.size(); i++)
    {
        wavetable.push_back(test_waveforms[i]);
    }

    // Test wavetable_was_received() - ad-hoc partial test
    // (N.B. private function, accessible because this test class is friend'ed
    // from the grid driver classes)
    t_fpu_state fpu_state;
    bool allow_unconfirmed = false;
    E_FPU_STATE target_state = FPST_READY_FORWARD;
    bool wtable_received_result =
            gd.wavetable_was_received(wavetable, 3, fpu_state,
                                      allow_unconfirmed, target_state);
    UNUSED_ARG(wtable_received_result); // Suppress variable-not-used warning

    //..........................................................................
    // Test GridDriver::getDuplicateSerialNumbers()
    const std::vector<std::string> test_snumbers =
    {
        // NOTE: Must be 6 chars or less (LEN_SERIAL_NUMBER - 1)
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
    for (size_t i = 0; i < test_snumbers.size(); i++)
    {
        // N.B. Using safer strncpy() (rather than strcpy()))
        strncpy(grid_state.FPU_state[i].serial_number,
                test_snumbers[i].c_str(), LEN_SERIAL_NUMBER);
        // Add guaranteed null-terminator at end of buffer
        grid_state.FPU_state[i].serial_number[LEN_SERIAL_NUMBER - 1] = '\0';
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
    
    //********************************************
    // Set the required parameters here
#ifndef FLEXIBLE_CAN_MAPPING // NOT FLEXIBLE_CAN_MAPPING
    const int num_fpus = 1;
#endif // NOT FLEXIBLE_CAN_MAPPING
    //const char *ip_address_str = "192.168.0.10";
    const char *ip_address_str = "127.0.0.1";
    //********************************************

#ifdef FLEXIBLE_CAN_MAPPING
    UnprotectedGridDriver ugd;
    ecan_result = ugd.initialize(test_can_map_path);
#else // NOT FLEXIBLE_CAN_MAPPING
    UnprotectedGridDriver ugd(num_fpus);

    ecan_result = ugd.initialize();
#endif // NOT FLEXIBLE_CAN_MAPPING

    if (ecan_result == DE_OK)
    {
        const uint16_t port_number = 4700;
        const t_gateway_address gateway_address = { ip_address_str, port_number };
        const bool protection_on = false;
#ifdef FLEXIBLE_CAN_MAPPING
        testInitialisedGridDriver(ugd, gateway_address, protection_on);
#else // NOT FLEXIBLE_CAN_MAPPING
        testInitialisedGridDriver(num_fpus, ugd, gateway_address, protection_on);
#endif // NOT FLEXIBLE_CAN_MAPPING
    }
}

//------------------------------------------------------------------------------
void GridDriverTester::doGridDriverFunctionalTesting()
{
    // Performs a grid driver functional test sequence
    
    E_EtherCANErrCode ecan_result;

    //********************************************
    // Set the required test parameters here
#ifndef FLEXIBLE_CAN_MAPPING // NOT FLEXIBLE_CAN_MAPPING
    const int num_fpus = 3;
#endif // NOT FLEXIBLE_CAN_MAPPING
    //const char *ip_address_str = "192.168.0.10"; // Good physical gateway
    const char *ip_address_str = "127.0.0.1";      // Local mock gateway
    const bool use_mockup_db = false;
    //********************************************

#ifdef FLEXIBLE_CAN_MAPPING
    GridDriver gd;
    ecan_result = gd.initialize(test_can_map_path);
#else // NOT FLEXIBLE_CAN_MAPPING
    GridDriver gd(num_fpus);

    ecan_result = gd.initialize();
#endif // NOT FLEXIBLE_CAN_MAPPING

    if (ecan_result == DE_OK)
    {
        ecan_result = gd.initProtection(use_mockup_db);
    }

    if (ecan_result == DE_OK)
    {
        const uint16_t port_number = 4700;
        const t_gateway_address gateway_address = { ip_address_str, port_number };
        const bool protection_on = true;
#ifdef FLEXIBLE_CAN_MAPPING
        testInitialisedGridDriver(gd, gateway_address, protection_on);
#else // NOT FLEXIBLE_CAN_MAPPING
        testInitialisedGridDriver(num_fpus, gd, gateway_address, protection_on);
#endif // NOT FLEXIBLE_CAN_MAPPING
    }
}

//------------------------------------------------------------------------------
#ifdef FLEXIBLE_CAN_MAPPING
void GridDriverTester::testInitialisedGridDriver(UnprotectedGridDriver &gd,
                                       const t_gateway_address &gateway_address,
                                                 bool protection_on)
#else // NOT FLEXIBLE_CAN_MAPPING
void GridDriverTester::testInitialisedGridDriver(int num_fpus,
                                                 UnprotectedGridDriver &gd,
                                       const t_gateway_address &gateway_address,
                                                 bool protection_on)
#endif // NOT FLEXIBLE_CAN_MAPPING
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

    //..........................................................................

#ifndef FLEXIBLE_CAN_MAPPING // NOT FLEXIBLE_CAN_MAPPING
    if (num_fpus > TESTING_MAX_NUM_FPUS)
    {
        return;
    }
#endif // NOT FLEXIBLE_CAN_MAPPING

    //..........................................................................

    volatile E_EtherCANErrCode ecan_result; // volatile so not optimised away,
                                            // so can see value in debugger
    E_GridState grid_state_result;
    t_grid_state gs;
    const bool support_uninitialized_auto = false;
    const bool soft_protection = protection_on;
    const bool count_protection = protection_on;
    // datum_timeout is DISABLED for now so that findDatum() doesn't time out,
    // which would otherwise do if the FPU arm(s) are far away from datum.
    // IMPORTANT: Be careful with this with physical FPUs - the datum timeout
    // is normally a safeguard.
    const E_DATUM_TIMEOUT_FLAG datum_timeout = DATUM_TIMEOUT_DISABLE;

    t_datum_search_flags search_modes;
    for (int fpu_id = 0; fpu_id < MAX_NUM_POSITIONERS; fpu_id++)
    {
        search_modes[fpu_id] = SEARCH_CLOCKWISE;
    }

    t_fpuset fpuset;
#ifdef FLEXIBLE_CAN_MAPPING
    const std::vector<int> &fpu_id_list = gd.config.getFpuIdList();
    UnprotectedGridDriver::createFpuSetForIdList(fpu_id_list, fpuset);
#else // NOT FLEXIBLE_CAN_MAPPING
    UnprotectedGridDriver::createFpuSetForNumFpus(num_fpus, fpuset);
#endif // NOT FLEXIBLE_CAN_MAPPING

    //..........................................................................
    // Test connect()
    // TODO: t_gateway_address::ip is only a pointer - dangerous? Change this
    // eventually? (e.g. to a std::string?)
    ecan_result = gd.connect(1, &gateway_address);   

    //..........................................................................
    if (ecan_result == DE_OK)
    {
        // Test getGridState()
        grid_state_result = gd.getGridState(gs);
    
        // Test readSerialNumbers()
        ecan_result = gd.readSerialNumbers(gs, fpuset);
    }
    
    //..........................................................................
    // Test pingFPUs()
    if (ecan_result == DE_OK)
    {
        ecan_result = gd.pingFPUs(gs, fpuset);
    
        // Test getGridState() again
        grid_state_result = gd.getGridState(gs);
    }

    // Test pingFPUs() again
    if (ecan_result == DE_OK)
    {
        ecan_result = gd.pingFPUs(gs, fpuset);
    }

    //..........................................................................
    // Test resetFPUs()
    if (ecan_result == DE_OK)
    {
        ecan_result = gd.resetFPUs(gs, fpuset);
    }
    
    //..........................................................................
    // Test findDatum()
    if (ecan_result == DE_OK)
    {
        grid_state_result = gd.getGridState(gs);

        ecan_result = gd.findDatum(gs, search_modes, DASEL_BOTH, fpuset,
                                   soft_protection, count_protection,
                                   support_uninitialized_auto, datum_timeout);
    }

#ifndef FLEXIBLE_CAN_MAPPING // NOT FLEXIBLE_CAN_MAPPING
    //..........................................................................
    // Test enableMove()
    grid_state_result = gd.getGridState(gs);

    for (int i = 0; i < num_fpus; i++)
    {
        ecan_result = gd.enableMove(i, gs);
        if (ecan_result != DE_OK)
        {
            break;
        }
    }
    
    //........................
    // N.B. If required, can uncomment the
    // following code to test the wtable
    // pruning code inside configMotion()
    /*    
    if (num_fpus >= 2)
    {
        fpuset[1] = false;
    }
    if (num_fpus >= 4)
    {
        fpuset[3] = false;
    }
    */ 
    //........................
#endif // NOT FLEXIBLE_CAN_MAPPING
    
    //..........................................................................
    // Test configMotion() / executeMotion() - a positive motion followed by
    // a negative one, which should bring us back to near datum
    if (ecan_result == DE_OK)
    {
        t_wtable wavetable;
        const bool allow_uninitialized = false;
        const int ruleset_version = DEFAULT_WAVEFORM_RULESET_VERSION;
        const bool warn_unsafe = true;
        const int verbosity = 3;

        //..............................
        // Test for (10,10)
        wavetable.push_back({0, getWaveform(GeneratedWaveform::Steps_10_10)});
        gd.getGridState(gs);
        ecan_result = gd.configMotion(wavetable, gs, fpuset,
                                      soft_protection, allow_uninitialized,
                                      ruleset_version, warn_unsafe, verbosity);
        if (ecan_result == DE_OK)
        {
            bool sync_command = true;
            gd.getGridState(gs);
            ecan_result = gd.executeMotion(gs, fpuset, sync_command);
        }

        //..............................
        // Test for (-9,-9)
        if (ecan_result == DE_OK)
        {
            wavetable.clear();
            wavetable.push_back({0, getWaveform(GeneratedWaveform::Steps_Minus9_Minus9)});
            gd.getGridState(gs);
            ecan_result = gd.configMotion(wavetable, gs, fpuset,
                                          soft_protection, allow_uninitialized,
                                          ruleset_version, warn_unsafe,
                                          verbosity);
        }
        if (ecan_result == DE_OK)
        {
            bool sync_command = true;
            gd.getGridState(gs);
            ecan_result = gd.executeMotion(gs, fpuset, sync_command);
        }

        //........................................
    }

    //..........................................................................
    // Test findDatum() again
    if (ecan_result == DE_OK)
    {
        gd.getGridState(gs);
        ecan_result = gd.findDatum(gs, search_modes, DASEL_BOTH, fpuset,
                                   soft_protection, count_protection,
                                   support_uninitialized_auto, datum_timeout);
    }
    
    //..........................................................................

    ecan_result = gd.disconnect();

    //..........................................................................

    // Suppress warnings of variables not being used
    UNUSED_ARG(grid_state_result);
}

//------------------------------------------------------------------------------
const t_waveform_steps &GridDriverTester::getWaveform(GeneratedWaveform gen_waveform)
{
    static const t_waveform_steps waveform_steps_none = {};
    
    // gen_wf(10,10)
    static const t_waveform_steps waveform_steps_10_10 =
    {
        { 62, 62}, {112, 112}, {162, 162}, {212, 162}, {193, 112},
        {162, 62}, {112,  62}, { 62,  62}, { 62,  47}
    };

    // gen_wf(-9,-9)
    static const t_waveform_steps waveform_steps_minus9_minus9 =
    {
        { -62,    0}, {-112,    0}, {-162, -62}, {-212, -112}, {-162, -162},
        {-112, -162}, { -62, -112}, { -62, -62}, {-62,   -62}, { -17,  -25}
    };

    // gen_wf(20,20)
    static const t_waveform_steps waveform_steps_20_20 =
    {
        { 62,   0}, {112,   0}, {162,   0}, {212,  62}, {250, 112},
        {250, 162}, {250, 212}, {250, 250}, {212, 250}, {162, 212},
        {112, 162}, { 62, 112}, {62,   62}, {62,   62}, { 58,  28}
    };

    // gen_wf(90,90)
    static const t_waveform_steps waveform_steps_90_90 =
    {
        { 62,   0}, {112,   0}, {162,   0}, {212,   0}, {250,   0},
        {250,   0}, {250,   0}, {250,   0}, {250,   0}, {250,   0},
        {250,  62}, {250, 112}, {250, 162}, {250, 212}, {250, 250},
        {250, 250}, {250, 250}, {250, 250}, {250, 250}, {250, 250},
        {250, 250}, {250, 250}, {250, 250}, {250, 250}, {250, 250},
        {250, 250}, {250, 250}, {250, 250}, {250, 250}, {250, 250},
        {250, 250}, {250, 250}, {250, 250}, {250, 250}, {250, 250},
        {250, 250}, {250, 250}, {250, 250}, {250, 250}, {250, 212},
        {212, 162}, {162, 112}, {112,  62}, { 62,  62}, { 62,  62},
        { 62,  62}, { 31,  56}
    };

    // gen_wf(-89,-89)
    static const t_waveform_steps waveform_steps_minus89_minus89 =
    {
        { -62,    0}, {-112,    0}, {-162,    0}, {-212,    0}, {-250,    0},
        {-250,    0}, {-250,    0}, {-250,    0}, {-250,    0}, {-250,    0},
        {-250,    0}, {-250,    0}, {-250,  -62}, {-250, -112}, {-250, -162},
        {-250, -212}, {-250, -250}, {-250, -250}, {-250, -250}, {-250, -250},
        {-250, -250}, {-250, -250}, {-250, -250}, {-250, -250}, {-250, -250},
        {-250, -250}, {-250, -250}, {-250, -250}, {-250, -250}, {-250, -250},
        {-250, -250}, {-250, -250}, {-250, -250}, {-250, -250}, {-250, -250},
        {-250, -250}, {-250, -250}, {-250, -250}, {-250, -250}, {-212, -250},
        {-162, -250}, {-112, -212}, { -62, -162}, { -62, -112}, { -62,  -62},
        { -62,  -62}, { -62,  -62}, { -43,  -34}
    };

    static const struct
    {
        GeneratedWaveform gen_waveform;
        t_waveform_steps waveform_steps;
    } waveform_defs[] = 
    {
        { GeneratedWaveform::Steps_10_10, waveform_steps_10_10 },
        { GeneratedWaveform::Steps_Minus9_Minus9, waveform_steps_minus9_minus9 },
        { GeneratedWaveform::Steps_20_20, waveform_steps_20_20 },
        { GeneratedWaveform::Steps_90_90, waveform_steps_90_90 },
        { GeneratedWaveform::Steps_Minus89_Minus89, waveform_steps_minus89_minus89 }
    };
    
    for (size_t i = 0; i < (sizeof(waveform_defs) / sizeof(waveform_defs[0])); i++)
    {
        if (waveform_defs[i].gen_waveform == gen_waveform)
        {
            return waveform_defs[i].waveform_steps;
        }
    }
    
    return waveform_steps_none;
}

//------------------------------------------------------------------------------
// TODO: Disabled because probably no longer needed - but keep for now
#if 0
//------------------------------------------------------------------------------
bool GridDriverTester::writeGridFpusToFpuDb(int num_fpus,
                                       const t_gateway_address &gateway_address,
                                            bool use_mockup_db)
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
    // The expected protection database location is controlled by the
    // use_mockup_db flag - see ProtectionDB::getDirFromLinuxEnv().
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
    UnprotectedGridDriver::createFpuSetForNumFpus(num_fpus, fpuset);
    
    //..........................................................................
    // Get grid FPU serial numbers using an UnprotectedGridDriver instance

#ifdef FLEXIBLE_CAN_MAPPING
    UnprotectedGridDriver ugd;
    ecan_result = ugd.initialize(test_can_map_path);
#else // NOT FLEXIBLE_CAN_MAPPING
    UnprotectedGridDriver ugd(num_fpus);
    ecan_result = ugd.initialize();
#endif // NOT FLEXIBLE_CAN_MAPPING
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

    std::string dir_str = ProtectionDB::getDirFromLinuxEnv(use_mockup_db);
    if (!dir_str.empty())
    {
        ProtectionDB protectiondb;
        if (protectiondb.open(dir_str) == MDB_SUCCESS)
        {
            for (size_t i = 0; i < serial_numbers.size(); i++)
            {
                result_ok = false;
                MdbResult mdb_result = MDB_PANIC;
                auto txn = protectiondb.createTransaction(mdb_result);
                if (txn)
                {
                    if (txn->fpuDbTransferFpu(DbTransferType::Write,
                                              serial_numbers[i].c_str(),
                                              fpu_db_data) == MDB_SUCCESS)
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
#endif // 0

//------------------------------------------------------------------------------
bool GridDriverTester::writeDummyFpuItemsToFpuDb(bool use_mockup_db,
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
    // The use_mockup_db flag controls the expected protection database
    // location - see ProtectionDB::getDirFromLinuxEnv().

    bool result_ok = false;
    std::string dir_str = ProtectionDB::getDirFromLinuxEnv(use_mockup_db);
    if (!dir_str.empty())
    {
        ProtectionDB protectiondb;
        if (protectiondb.open(dir_str) == MDB_SUCCESS)
        {
            MdbResult mdb_result = MDB_PANIC;
            auto txn = protectiondb.createTransaction(mdb_result);
            if (txn)
            {
                FpuDbData fpu_db_data;
                ProtectionDBTester::fillFpuDbDataStructWithTestVals(fpu_db_data);
                if (txn->fpuDbTransferFpu(DbTransferType::Write, serial_number,
                                          fpu_db_data) == MDB_SUCCESS)
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
