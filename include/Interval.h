// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-06-12  TODO: Put comment here
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME Interval.h
//
// TODO: Put description here
//
////////////////////////////////////////////////////////////////////////////////

#ifndef INTERVAL_H
#define INTERVAL_H

namespace mpifps
{

//..............................................................................
    
class Interval
{
public:
    Interval()
    {
        // TODO: Default to NAN instead 0.0?
        lower = 0.0;
        upper = 0.0;
    }

    Interval(double lower, double upper)
    {
        this->lower = lower;
        this->upper = upper;
    }

private:
    double lower;
    double upper;
};

//..............................................................................

} // namespace mpifps

#endif // INTERVAL_H
