// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-08-05  TODO: Put comment here
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME GridDriverTester.h
//
// TODO: Put description here
//
////////////////////////////////////////////////////////////////////////////////

#ifndef GRIDDRIVERTESTER_H
#define GRIDDRIVERTESTER_H

#include "UnprotectedGridDriver.h"
#include "GridDriver.h"

namespace mpifps
{

//==============================================================================

class GridDriverTester
{
    // TODO: Will this actually be the case?
    // N.B. This class is friend-ed from UnprotectedGridDriver, so it can
    // access its private and protected member variables and functions
public:
    void testUnprotectedGridDriver();
    void testGridDriver();
    
private:
    void doTests(UnprotectedGridDriver &grid_driver);

    void test_initialize();
    void test_connect();
    void test_findDatum();
    void test_configMotion();
    void test_executeMotion();
};

//==============================================================================

} // namespace mpifps

#endif // GRIDDRIVERTESTER_H