// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-08-05  Created.
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME GridDriverTester.h
//
// Provides test functionality for testing the UnprotectedGridDriver and
// GridDriver classes.
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
    void doGridDriverUnitTests();

    void doUnprotectedGridDriverFunctionalTesting();
    void doGridDriverFunctionalTesting();
    bool writeGridFpusToFpuDb(int num_fpus, bool db_mockup);
    static bool writeDummyFpuItemsToFpuDb(bool db_mockup,   // N.B. static
                                          const char *serial_number);

private:
    void testInitialisedGridDriver(UnprotectedGridDriver &gd,
                                   bool soft_protection);
    
    const char *ip_address_str = "127.0.0.1";
    const uint16_t port_number = 4700;
    const t_gateway_address gateway_address = { ip_address_str, port_number };
};

//==============================================================================

} // namespace mpifps

#endif // GRIDDRIVERTESTER_H