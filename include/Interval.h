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
    }

    Interval(double scalar)
    {
        lower = scalar;
        upper = scalar;
    }
    
    Interval(double lower_in, double upper_in)
    {
      // TODO: Check for / enforce lower/upper being assigned actual lower/upper
      // values?
      
        lower = lower_in;
        upper = upper_in;
    }

    void getLowerUpper(double &lower_ret, double &upper_ret)
    {
        lower_ret = lower;
        upper_ret = upper;
    }

private:
    // TODO: Initialise to NAN instead 0.0?
    double lower = 0.0;
    double upper = 0.0;
};

//..............................................................................

} // namespace mpifps

#endif // INTERVAL_H
