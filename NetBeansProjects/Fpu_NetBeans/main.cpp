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
#include "GridDriver.h"
#include "GridDriverTester.h"

using namespace std;

static void testGridDriver();
static void testFpuCounters();
static void testProtectionDB();


//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    testGridDriver();

    //testGridDriver();

    //testFpuCounters();
    
    //testProtectionDB();
    
    return 0;
}

//------------------------------------------------------------------------------
static void testGridDriver()
{
    GridDriverTester grid_driver_tester;

    grid_driver_tester.doGridDriverUnitTests();
    
#if 0
    grid_driver_tester.testUnprotectedGridDriver();
#else
    grid_driver_tester.doGridDriverFunctionalTesting();
#endif
}

//------------------------------------------------------------------------------
static void testFpuCounters()
{
    FpuCounters fpu_counters;
    size_t num_bytes;
    
    void *raw_data_ptr = fpu_counters.getRawData(num_bytes);
}

//------------------------------------------------------------------------------
static void testProtectionDB()
{
    bool result_ok = protectionDB_Test();
}

//------------------------------------------------------------------------------
