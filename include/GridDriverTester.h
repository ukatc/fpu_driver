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

namespace mpifps
{

//==============================================================================

class UnprotectedGridDriverTester
{
    // TODO: No longer friend-ed? If not then remove this comment
    // N.B. This class is friend-ed from UnprotectedGridDriver, so it can
    // access its private and protected member variables and functions
public:
    void doTests();
    
private:
    void test_initialize();
    void test_connect();
    void test_findDatum();
    void test_configMotion();
    void test_executeMotion();
};

//==============================================================================

} // namespace mpifps

#endif // GRIDDRIVERTESTER_H