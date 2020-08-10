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
#include "UnprotectedGridDriver.h"
#include "GridDriver.h"
#include "E_GridState.h"
#include "InterfaceState.h"

#define DOTESTS_MAX_NUM_FPUS    (5)

// NOTE: Can change DOTESTS_NUM_FPUS as required, up to DOTESTS_MAX_NUM_FPUS -
// and set mock gateway (if used) to the same value when invoking it
#define DOTESTS_NUM_FPUS        (3)
#if (DOTESTS_NUM_FPUS > DOTESTS_MAX_NUM_FPUS)
#error "BUILD ERROR: DOTESTS_NUM_FPUS > DO_TEST_MAX_NUM_FPUS"
#endif

namespace mpifps
{
    
//------------------------------------------------------------------------------
void UnprotectedGridDriverTester::doTests()
{
    // Allows basic fixed-argument testing of the main UnprotectedGridDriver()
    // functions, for up to 5 FPUs. Notes:
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
    const bool soft_protection = false;

    //..........................................................................

    UnprotectedGridDriver ugd(DOTESTS_NUM_FPUS);

    //..........................................................................
    // Test initialize()
    result = ugd.initialize();    

    //..........................................................................
    // Test connect()
    // TODO: t_gateway_address::ip is only a pointer - dangerous? Change this
    // eventually? (e.g. to a std::string?)
    const char *ip_address_str = "127.0.0.1";
    uint16_t port_number = 4700;
    t_gateway_address gateway_address = { ip_address_str, port_number };
    result = ugd.connect(1, &gateway_address);   

    //..........................................................................
    // Specify FPUs in fpuset
    t_fpuset fpuset;
    for (int fpu_id = 0; fpu_id < MAX_NUM_POSITIONERS; fpu_id++)
    {
        fpuset[fpu_id] = false;
    }
    for (int fpu_id = 0; fpu_id < DOTESTS_NUM_FPUS; fpu_id++)
    {
        fpuset[fpu_id] = true;
    }

    //..........................................................................
    // Test getGridState()
    grid_state_result = ugd.getGridState(grid_state);

    //..........................................................................
    // Test pingFPUs()
    result = ugd.pingFPUs(grid_state, fpuset);
    
    //..........................................................................
    // Test getGridState() again
    grid_state_result = ugd.getGridState(grid_state);

    //..........................................................................
    // Test pingFPUs() again
    result = ugd.pingFPUs(grid_state, fpuset);

    //..........................................................................
    // Test resetFPUs()
    result = ugd.resetFPUs(grid_state, fpuset);
    
    //..........................................................................
    // Test findDatum()
    t_datum_search_flags search_modes;
    for (int fpu_id = 0; fpu_id < DOTESTS_NUM_FPUS; fpu_id++)
    {
        search_modes[fpu_id] = SEARCH_CLOCKWISE;
    }
    const bool count_protection = false;
    const bool support_uninitialized_auto = false;

    grid_state_result = ugd.getGridState(grid_state);

    result = ugd.findDatum(grid_state, search_modes, DASEL_BOTH, fpuset,
                           soft_protection, count_protection,
                           support_uninitialized_auto, DATUM_TIMEOUT_ENABLE);

    //..........................................................................
    // Test configMotion() and wavetable_was_received()
    const bool allow_uninitialized = true;
    const int ruleset_version = DEFAULT_WAVEFORM_RULESET_VERSION;
    const bool warn_unsafe = true;
    const int verbosity = 3;

    t_wtable wavetable;
    static const t_waveform test_waveforms[DOTESTS_MAX_NUM_FPUS] =
    {
        {0, { { 0,  -1}, { 2,  -3}, { 4,  -5} } },
        {1, { { 6,  -7}, { 8,  -9}, {10, -11} } },
        {2, { {12, -13}, {14, -15}, {16, -17} } },
        {3, { {18, -19}, {20, -21}, {22, -23} } },
        {4, { {24, -25}, {26, -27}, {28, -29} } }
    };
    for (int i = 0; i < DOTESTS_NUM_FPUS; i++)
    {
        wavetable.push_back(test_waveforms[i]);
    }

    // Ad-hoc partial test of wavetable_was_received()
    t_fpu_state fpu_state;
    bool allow_unconfirmed = false;
    E_FPU_STATE target_state = FPST_READY_FORWARD;
    
    //*********** TODO: Reinstate? Requires UnprotectedGridDriverTester to be
    // re-friend-ed by UnprotectedGridDriver (and GridDriver?)
//#define FRIEND_OF_TESTED_CLASS
#ifdef FRIEND_OF_TESTED_CLASS
    bool wtable_received_result = ugd.wavetable_was_received(wavetable, 3, 
                                                             fpu_state,
                                                             allow_unconfirmed,
                                                             target_state);
#endif    
    
    grid_state_result = ugd.getGridState(grid_state);

    for (int i = 0; i < DOTESTS_NUM_FPUS; i++)
    {
        result = ugd.enableMove(i, grid_state);
    }
    
    //........................
    // N.B. If required, can uncomment the
    // following code to test the wtable
    // pruning code inside configMotion()
    /*    
    if (DOTESTS_NUM_FPUS >= 2)
    {
        fpuset[1] = false;
    }
    if (DOTESTS_NUM_FPUS >= 4)
    {
        fpuset[3] = false;
    }
    */ 
    //........................
    
    result = ugd.configMotion(wavetable, grid_state, fpuset, soft_protection,
                              allow_uninitialized, ruleset_version, 
                              warn_unsafe, verbosity);

    //..........................................................................
    // Test executeMotion()
    bool sync_command = true;
    result = ugd.executeMotion(grid_state, fpuset, sync_command);
    
    //..........................................................................

    result = ugd.disconnect();

    //..........................................................................

    // Suppress warnings of variables not being used
    UNUSED_ARG(result);
    UNUSED_ARG(grid_state_result);
#ifdef FRIEND_OF_TESTED_CLASS    
    UNUSED_ARG(wtable_received_result);
#endif // FRIENDED_CLASS    
}

//------------------------------------------------------------------------------
void UnprotectedGridDriverTester::test_initialize()
{

    
}

//------------------------------------------------------------------------------
void UnprotectedGridDriverTester::test_connect()
{

    
}

//------------------------------------------------------------------------------
void UnprotectedGridDriverTester::test_findDatum()
{


}

//------------------------------------------------------------------------------
void UnprotectedGridDriverTester::test_configMotion()
{
    
}

//------------------------------------------------------------------------------
void UnprotectedGridDriverTester::test_executeMotion()
{
    
}

} // namespace mpifps
