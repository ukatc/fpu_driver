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
    void doGridDriverUnitTests();

    void doUnprotectedGridDriverFunctionalTesting();
    void doGridDriverFunctionalTesting();
    bool writeGridFpusToFpuDb(int num_fpus, bool db_mockup);

private:
    void testInitialisedGridDriver(UnprotectedGridDriver &gd,
                                   bool soft_protection);
#ifdef FPU_DB_DATA_AGGREGATED
    void writeFpuDbDummyItemsFromSerialNumbers(GridDriver &gd);
    
    const char *ip_address_str = "127.0.0.1";
    const uint16_t port_number = 4700;
    const t_gateway_address gateway_address = { ip_address_str, port_number };
#endif
};

//==============================================================================

} // namespace mpifps

#endif // GRIDDRIVERTESTER_H