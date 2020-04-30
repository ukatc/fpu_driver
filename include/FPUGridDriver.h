// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-04-28  TODO: Put comment here
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME FPUGridDriver.h
//
// TODO: Put description here
//
////////////////////////////////////////////////////////////////////////////////

#ifndef FPUGRIDDRIVER_H
#define FPUGRIDDRIVER_H

namespace mpifps
{

class UnprotectedGridDriver
{
public:
    UnprotectedGridDriver();

    // TODO: Ad-hoc test function only - remove when no longer needed
    int testFunction();

private:
    // TODO: Ad-hoc test variable only - remove when no longer needed
    int dummyCounter = 0;

};

class GridDriver
{
public:
    GridDriver();

private:    

};


} // namespace mpifps

#endif // FPUGRIDDRIVER_H
