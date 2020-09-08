// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-08-27  TODO: Put comment here
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME ProtectionDBTester.h
//
// TODO: Put description here
//
////////////////////////////////////////////////////////////////////////////////

#ifndef PROTECTIONDBTESTER_H
#define PROTECTIONDBTESTER_H

#include "ProtectionDB.h"
#include "GridDriver.h"

using namespace mpifps;

bool protectionDB_Test();

class ProtectionDBTester
{
    // N.B. This class is friend-ed from ProtectionDB, so it can access its
    // private and protected member variables and functions for test purposes

public:
    void writeFpuDbTestItemsFromSerialNumbers(GridDriver &gd);
    bool testFpuDbDataClass();
};

#endif // PROTECTIONDBTESTER_H
