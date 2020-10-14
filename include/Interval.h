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
    Interval();
    Interval(double scalar);
    Interval(double lower_in, double upper_in);

    void getLowerUpper(double &lower_ret, double &upper_ret);
    bool operator==(const Interval &other);
    bool operator!=(const Interval &other);
    void operator+=(double val);
    Interval operator+(double val);
    Interval operator-(double val);
    void assignCombine(const Interval &otherInterval);
    bool contains(const Interval &otherInterval, double tolerance) const;

private:
    // TODO: Initialise to NAN instead 0.0?
    double lower = 0.0;
    double upper = 0.0;
};

// Simple Interval class test function
void testIntervalClass(void);

//..............................................................................

} // namespace mpifps

#endif // INTERVAL_H
