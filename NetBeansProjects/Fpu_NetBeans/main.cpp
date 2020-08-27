/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

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
    
    if (interval_1 == interval_2)
    {
        dummy_val++;
    }
    
    if (interval_1 != interval_3)
    {
        dummy_val++;
    }
    
    if (interval_1 == interval_4)
    {
        dummy_val++;
    }
    
    if (interval_1 != interval_4)
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

    // gd_tester.doGridDriverUnitTests();
    
#if 0
    gd_tester.testUnprotectedGridDriver();
#else
    gd_tester.doGridDriverFunctionalTesting();
#endif
}

//------------------------------------------------------------------------------
static void testProtectionDB()
{
    bool result_ok = protectionDB_Test();
}

//------------------------------------------------------------------------------
