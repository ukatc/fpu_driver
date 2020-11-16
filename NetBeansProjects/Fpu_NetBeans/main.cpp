// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-05-25  Created.
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME main.cpp
//
// Top-level main file for grid driver testing purposes.
//
////////////////////////////////////////////////////////////////////////////////

/* 
 * File:   main.cpp
 * Author: bartw
 *
 * Created on 25 May 2020, 14:02
 */

#include <cstdlib>
#include "ProtectionDB.h"
#include "ProtectionDBTester.h"
#include "GridDriver.h"
#include "GridDriverTester.h"
#include "Interval.h"
#include "FPUCounters.h"

using namespace std;

static void testGridDriver();
static void testProtectionDB();

//*****************************
// TODO: For testing only
//#define GET_STACK_SIZE
#ifdef GET_STACK_SIZE
#include <sys/resource.h>
#endif
//*****************************

//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    //testIntervalClass();
    
    // Test code for writing dummy items to the FPU database for the "PT19" FPU,
    // which is the serial number of the physical test FPU
#if 0
    const bool db_mockup = true;
    bool result_ok = GridDriverTester::writeDummyFpuItemsToFpuDb(db_mockup,
                                                                 "PT19");
#endif // 0
    
    testGridDriver();

    //testFpuCounters();

    //testProtectionDB();
    

//*****************************
// TODO: For testing only
#ifdef GET_STACK_SIZE
    struct rlimit rl;
    int result = getrlimit(RLIMIT_STACK, &rl);
#endif
//*****************************    

    return 0;
}

//------------------------------------------------------------------------------
static void testGridDriver()
{
    GridDriverTester gd_tester;

/*
    bool db_mockup = true;
    int num_fpus = 3;
    bool result_ok = gd_tester.writeGridFpusToFpuDb(num_fpus, db_mockup);
*/

    // gd_tester.doGridDriverUnitTests();
    
#if 0
    //gd_tester.doUnprotectedGridDriverFunctionalTesting();
#else
    gd_tester.doGridDriverFunctionalTesting();
#endif
}

//------------------------------------------------------------------------------
static void testProtectionDB()
{
    //bool result_ok = protectionDB_Test();
    bool result_ok = ProtectionDBTester::doLoopingTestsWithConsoleOutput();
}

//------------------------------------------------------------------------------
