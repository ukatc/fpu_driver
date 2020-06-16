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

using namespace std;

static void testUnprotectedGridDriver();
static void testGridDriver();
static void testFpuCounters();
static void testProtectionDB();


//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    testUnprotectedGridDriver();

    //testGridDriver();

    //testFpuCounters();
    
    //testProtectionDB();
    
    return 0;
}

//------------------------------------------------------------------------------

static void testUnprotectedGridDriver()
{
    UnprotectedGridDriverTester ugd_tester;
    
    ugd_tester.doTests();
}

static void testGridDriver()
{
    GridDriver grid_driver;
    
    double blah = grid_driver.boostPythonDivide(2.0, 1.0);
}

static void testFpuCounters()
{
    FpuCounters fpu_counters;
    size_t num_bytes;
    
    void *raw_data_ptr = fpu_counters.getRawData(num_bytes);
}

static void testProtectionDB()
{
    bool result_ok = protectionDB_Test();
}

//------------------------------------------------------------------------------
