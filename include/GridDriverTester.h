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
    // N.B. This class is friend-ed from UnprotectedGridDriver and GridDriver,
    // so it can access their private and protected member variables and
    // functions for test purposes
public:
    void testUnprotectedGridDriver();
    void testGridDriver();
    
private:
    void testInitialisedGridDriver(UnprotectedGridDriver &grid_driver,
                                   bool soft_protection);
};

//==============================================================================

} // namespace mpifps

#endif // GRIDDRIVERTESTER_H