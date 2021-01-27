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
    bool writeGridFpusToFpuDb(int num_fpus,
                              const t_gateway_address &gateway_address,
                              bool use_mockup_db);
    static bool writeDummyFpuItemsToFpuDb(bool use_mockup_db,   // N.B. static
                                          const char *serial_number);

private:
    enum class GeneratedWaveform
    {
        Steps_10_10,
        Steps_20_20,
        Steps_90_90,
        Steps_Minus89_Minus89
    };
    void testInitialisedGridDriver(UnprotectedGridDriver &gd,
                                   const t_gateway_address &gateway_address,
                                   bool soft_protection);
    const t_waveform_steps &getWaveform(GeneratedWaveform gen_waveform);
};

//==============================================================================

} // namespace mpifps

#endif // GRIDDRIVERTESTER_H
