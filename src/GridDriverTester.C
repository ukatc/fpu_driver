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

#include "GridDriverTester.h"
#include "E_GridState.h"
#include "InterfaceState.h"

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
void GridDriverTester::testUnprotectedGridDriver()
{
    E_EtherCANErrCode result;
    
    UnprotectedGridDriver ugd(TESTING_NUM_FPUS);

    result = ugd.initialize();

    if (result == DE_OK)
    {
        const bool soft_protection = false;
        testInitialisedGridDriver(ugd, soft_protection);
    }
}

//------------------------------------------------------------------------------
void GridDriverTester::testGridDriver()
{
    E_EtherCANErrCode result;
    
    GridDriver gd(TESTING_NUM_FPUS);

    result = gd.initialize();

    if (result == DE_OK)
    {
        const bool mockup = true;
        result = gd.initProtection(mockup);
    }

    if (result == DE_OK)
    {
        const bool soft_protection = true;
        testInitialisedGridDriver(gd, soft_protection);
    }
}

//------------------------------------------------------------------------------
void GridDriverTester::testInitialisedGridDriver(UnprotectedGridDriver &grid_driver,
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

    E_EtherCANErrCode result;
    E_GridState grid_state_result;
    t_grid_state grid_state;

    //..........................................................................
    // Test connect()
    // TODO: t_gateway_address::ip is only a pointer - dangerous? Change this
    // eventually? (e.g. to a std::string?)
    const char *ip_address_str = "127.0.0.1";
    uint16_t port_number = 4700;
    t_gateway_address gateway_address = { ip_address_str, port_number };
    result = grid_driver.connect(1, &gateway_address);   

    //..........................................................................
    // Specify FPUs in fpuset
    t_fpuset fpuset;
    for (int fpu_id = 0; fpu_id < MAX_NUM_POSITIONERS; fpu_id++)
    {
        fpuset[fpu_id] = false;
    }
    for (int fpu_id = 0; fpu_id < TESTING_NUM_FPUS; fpu_id++)
    {
        fpuset[fpu_id] = true;
    }

    //..........................................................................
    // Test getGridState()
    grid_state_result = grid_driver.getGridState(grid_state);
    
    //..........................................................................
    // Test readSerialNumbers()
    result = grid_driver.readSerialNumbers(grid_state, fpuset);
    
    //..........................................................................
    // Test pingFPUs()
    result = grid_driver.pingFPUs(grid_state, fpuset);
    
    //..........................................................................
    // Test getGridState() again
    grid_state_result = grid_driver.getGridState(grid_state);

    //..........................................................................
    // Test pingFPUs() again
    result = grid_driver.pingFPUs(grid_state, fpuset);

    //..........................................................................
    // Test resetFPUs()
    result = grid_driver.resetFPUs(grid_state, fpuset);
    
    //..........................................................................
    // Test findDatum()
    t_datum_search_flags search_modes;
    for (int fpu_id = 0; fpu_id < TESTING_NUM_FPUS; fpu_id++)
    {
        search_modes[fpu_id] = SEARCH_CLOCKWISE;
    }
    const bool count_protection = false;
    const bool support_uninitialized_auto = false;

    grid_state_result = grid_driver.getGridState(grid_state);

    // TODO: Use DATUM_TIMEOUT_DISABLE instead of DATUM_TIMEOUT_ENABLE below?
    // (sometimes times out if long-duration findDatum())
    result = grid_driver.findDatum(grid_state, search_modes, DASEL_BOTH, fpuset,
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
            grid_driver.wavetable_was_received(wavetable, 3, fpu_state,
                                               allow_unconfirmed, target_state);
    UNUSED_ARG(wtable_received_result); // Suppress variable-not-used warning
    
    grid_state_result = grid_driver.getGridState(grid_state);

    for (int i = 0; i < TESTING_NUM_FPUS; i++)
    {
        result = grid_driver.enableMove(i, grid_state);
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
    
    result = grid_driver.configMotion(wavetable, grid_state, fpuset,
                                      soft_protection, allow_uninitialized,
                                      ruleset_version, warn_unsafe, verbosity);

    //..........................................................................
    // Test executeMotion()
    bool sync_command = true;
    result = grid_driver.executeMotion(grid_state, fpuset, sync_command);
    
    //..........................................................................

    result = grid_driver.disconnect();

    //..........................................................................

    // Suppress warnings of variables not being used
    UNUSED_ARG(result);
    UNUSED_ARG(grid_state_result);
}

//------------------------------------------------------------------------------

} // namespace mpifps
