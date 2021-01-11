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
    static MdbResult testFpuIntervalTransfer(ProtectionDB &protectiondb);
    static MdbResult testFpuCountersTransfer(ProtectionDB &protectiondb);
    static MdbResult testFpuWaveformTransfer(ProtectionDB &protectiondb);
    static MdbResult testFpuInt64ValTransfer(ProtectionDB &protectiondb);
    static MdbResult testFpuWfReversedFlagTransfer(ProtectionDB &protectiondb);
    static MdbResult testFullFpuDataTransfer(ProtectionDB &protectiondb);

    // FPU database binary-level item transfer test functions
    static MdbResult testFpuSingleItemWriteRead(ProtectionDB &protectiondb);
    static MdbResult testFpuMultipleItemWriteReads(ProtectionDB &protectiondb);

    // Miscellaneous test functions
    static std::string getRandomFpuTestSerialNumber();
};

#endif // PROTECTIONDBTESTER_H
