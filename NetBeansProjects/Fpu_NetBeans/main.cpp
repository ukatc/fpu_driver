// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-05-25  TODO: Put comment here
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


//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    //testIntervalClass();
    
    //testGridDriver();

    //testFpuCounters();

    testProtectionDB();
    

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
    bool result_ok = protectionDB_LoopingTestWithConsoleOutput();
}

//------------------------------------------------------------------------------
