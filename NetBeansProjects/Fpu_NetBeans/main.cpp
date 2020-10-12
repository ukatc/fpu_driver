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
    //..........................................................................
    // Ad-hoc testing of Interval class
    Interval interval_1(1.1, 1.2);
    Interval interval_2(1.1, 9.9);
    Interval interval_3(3.5, 3.6);
    Interval interval_4(1.1, 1.2);
    int dummy_val = 0;

    // Test == operator overload
    if (interval_1 == interval_2)
    {
        dummy_val++;
    }
    
    // Test != operator overload
    if (interval_1 != interval_3)
    {
        dummy_val++;
    }
    
    // Test == operator overload
    if (interval_1 == interval_4)
    {
        dummy_val++;
    }
    
    // Test != operator overload
    if (interval_1 != interval_4)
    {
        dummy_val++;
    }
    
    // Test - operator overload
    Interval interval_5 = interval_1 - 0.5;
    
    // Test + operator overload
    Interval interval_6(23.0, 34.0);
    Interval interval_7 = interval_6 + 10.0;
    
    // Test contains()
    Interval interval_8(8.0, 22.0);
    Interval interval_9(10.0, 20.0);
    if (interval_9.contains(interval_8, 2.0))
    {
        dummy_val++;
    }
    if (interval_9.contains(interval_8, 2.5))
    {
        dummy_val++;
    }
    if (interval_9.contains(interval_8, 1.999))
    {
        dummy_val++;
    }
    
    
    //..........................................................................
    
    //testGridDriver();

    //testFpuCounters();

    testProtectionDB();
    
    //testProtectionDB();
    
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
