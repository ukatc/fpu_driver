// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-10-12  TODO: Put comment here
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME Interval.C
//
// This Interval class represents values as intervals with upper and lower
// bounds values.
//
////////////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include "Interval.h"

namespace mpifps
{

//==============================================================================
Interval::Interval()
{
}

//------------------------------------------------------------------------------
Interval::Interval(double scalar)
{
    lower = scalar;
    upper = scalar;
}

//------------------------------------------------------------------------------
Interval::Interval(double lower_in, double upper_in)
{
    // TODO: Check for / enforce lower/upper being assigned actual lower/upper
    // values?
    
    lower = lower_in;
    upper = upper_in;
}

//------------------------------------------------------------------------------
void Interval::getLowerUpper(double &lower_ret, double &upper_ret)
{
    lower_ret = lower;
    upper_ret = upper;
}

//------------------------------------------------------------------------------
bool Interval::operator==(const Interval &other)
{
    return ((lower == other.lower) && (upper == other.upper));
}

//------------------------------------------------------------------------------
bool Interval::operator!=(const Interval &other)
{
    return ((lower != other.lower) || (upper != other.upper));
}

//------------------------------------------------------------------------------
void Interval::operator+=(double val)
{
    // TODO: Is this operation equivalent to the Python version of
    // Interval.__add__()? (which is called from e.g.
    // GridDriver._post_connect_hook() I think, in the line:
    // in_dicts[subkey][fpu_id] = val + alpha_datum_offset
    lower += val;
    upper += val;
}

//------------------------------------------------------------------------------
Interval Interval::operator+(double val)
{
    // TODO: Is this addition operation correct?
    // N.B. It's used in e.g. GridDriver::_refresh_positions()
    Interval interval_ret;
    interval_ret.lower = lower + val;
    interval_ret.upper = upper + val;
    return interval_ret;
}

//------------------------------------------------------------------------------
Interval Interval::operator-(double val)
{
    // TODO: Is this subtraction operation correct?
    // N.B. It's used in e.g. GridDriver::_reset_hook()
    Interval interval_ret;
    interval_ret.lower = lower - val;
    interval_ret.upper = upper - val;
    return interval_ret;
}

//------------------------------------------------------------------------------
void Interval::assignCombine(const Interval &otherInterval)
{
    // Combines otherInterval into this interval
    lower = std::min(lower, otherInterval.lower);
    upper = std::max(upper, otherInterval.upper);
}

//------------------------------------------------------------------------------
bool Interval::contains(const Interval &otherInterval, double tolerance) const
{
    // N.B. tolerance must be a positive value
    if ( ((lower - tolerance) <= otherInterval.lower) &&
         ((upper + tolerance) >= otherInterval.upper) )
    {
        return true;
    }
    return false;
}

//==============================================================================
void testIntervalClass(void)
{
    // Ad-hoc Interval test function - single-step in debugger and check the
    // results
    Interval interval_1(1.1, 1.2);
    Interval interval_2(1.1, 9.9);
    Interval interval_3(3.5, 3.6);
    Interval interval_4(1.1, 1.2);
    int dummy_val = 0;

    // Test == operator overload
    if (interval_1 == interval_2)
    {
        dummy_val++;
    }
    
    // Test != operator overload
    if (interval_1 != interval_3)
    {
        dummy_val++;
    }
    
    // Test == operator overload
    if (interval_1 == interval_4)
    {
        dummy_val++;
    }
    
    // Test != operator overload
    if (interval_1 != interval_4)
    {
        dummy_val++;
    }
    
    // Test - operator overload
    Interval interval_5 = interval_1 - 0.5;
    
    // Test + operator overload
    Interval interval_6(23.0, 34.0);
    Interval interval_7 = interval_6 + 10.0;

    // Test assignCombine()
    Interval interval_8(-2.34, 6.5);
    Interval interval_9(-3.95, 5.0);
    interval_8.assignCombine(interval_9);
    Interval interval_10(-2.5, 10.77);
    interval_8.assignCombine(interval_10);
    
    // Test contains()
    Interval interval_11(8.0, 22.0);
    Interval interval_12(10.0, 20.0);
    if (interval_12.contains(interval_11, 2.0))
    {
        dummy_val++;
    }
    if (interval_12.contains(interval_11, 2.5))
    {
        dummy_val++;
    }
    if (interval_12.contains(interval_11, 1.999))
    {
        dummy_val++;
    }
}

//==============================================================================


} // namespace mpifps