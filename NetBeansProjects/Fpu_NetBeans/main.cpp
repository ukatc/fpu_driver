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
#include "FPUGridDriver.h"

using namespace std;

static void testFpuGridDriver();
static void testFpuCounters();
static void testProtectionDB();


//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    testFpuGridDriver();
    
    //testFpuCounters();
    
    //testProtectionDB();
    
    return 0;
}

//------------------------------------------------------------------------------
static void testFpuGridDriver()
{
    UnprotectedGridDriver unprotected_grid_driver;

    // TODO: NOTE: I don't yet know what format of IP address is expected - just
    // using a dummy format for now
    // TODO: Also, t_gateway_address::ip is only a pointer - dangerous? Change
    // this eventually? (e.g. to a std::String?)
    const char *dummy_ip_str = "192.168.12.34";
    t_gateway_address gateway_address = { dummy_ip_str, 12345 };

    unprotected_grid_driver.connect(1, &gateway_address);
    
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
