// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-08-27  Created.
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME ProtectionDBTester.h
//
// Provides test functionality for the ProtectionDB grid driver protection
// database classes.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef PROTECTIONDBTESTER_H
#define PROTECTIONDBTESTER_H

#include <string>
#include "ProtectionDB.h"
#include "GridDriver.h"

using namespace mpifps;

class ProtectionDBTester
{
    // N.B. This class is friend-ed from ProtectionDB, so it can access its
    // private and protected member variables and functions for test purposes.

    // N.B. The functions in this class are currently declared as static
    // because they don't require any private data - this class is just a
    // convenient test function aggregator for now.

public:
    static bool doLoopingTestsWithConsoleOutput();
    static bool doTests();
    static void fillFpuDbDataStructWithTestVals(FpuDbData &fpu_db_data);
    static bool testFpuDbDataClass();
    static void testGetSerialNumFromKeyVal();   // Ad-hoc testing
    static void testDbOpeningScenarios();  // Ad-hoc testing

private:
    // Top-level test functions
    static bool testWithStayingOpen(const std::string &dir_str);
    static bool testWithClosingReopening(const std::string &dir_str);

    // FPU database data item transfer test functions
    static bool testFpuIntervalTransfer(ProtectionDB &protectiondb);
    static bool testFpuCountersTransfer(ProtectionDB &protectiondb);
    static bool testFpuWaveformTransfer(ProtectionDB &protectiondb);
    static bool testFpuInt64ValTransfer(ProtectionDB &protectiondb);
    static bool testFpuWfReversedFlagTransfer(ProtectionDB &protectiondb);
    static bool testFullFpuDataTransfer(ProtectionDB &protectiondb);

    // FPU database binary-level item transfer test functions
    static bool testFpuSingleItemWriteRead(ProtectionDB &protectiondb);
    static bool testFpuMultipleItemWriteReads(ProtectionDB &protectiondb);

    // Miscellaneous test functions
    static std::string getRandomFpuTestSerialNumber();
};

#endif // PROTECTIONDBTESTER_H
