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

using namespace std;

/*
 * 
 */
int main(int argc, char** argv)
{
    //..........................................................................
    // Test FpuCounters class
    FpuCounters fpu_counters;
    
    size_t num_bytes;
    void *raw_data_ptr = fpu_counters.getRawData(num_bytes);
    
    //..........................................................................
    // Test ProtectionDB class
    
    bool result_ok = protectionDB_Test();
   
    
    //..........................................................................
    
    return 0;
}

