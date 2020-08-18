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
static void testProtectionDB();


//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    //testGridDriver();

    testFpuCounters();
    
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
