// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-10-12  Created (translated from Python interval.py).
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME Interval.C
//
// This Interval class represents values as intervals with upper and lower
// bounds values.
//
////////////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <cmath>
#include <string>
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
std::string Interval::toString()
{
    // TODO: This function uses std::to_string() on the double values,
    // which produces fixed 6-decimal-place output - reduce number of decimal
    // places using e.g. std::stringstream's?
    
    std::string return_string;

    if (std::isnan(lower) && std::isnan(upper))
    {
        return_string = "[]";
    }
    else
    {
        return_string = "[" + std::to_string(lower) + ", " + 
                              std::to_string(upper) + "]";
    }

    return return_string;
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
Interval Interval::operator+(const Interval &intervalToAdd)
{
    // TODO: Is this interval+interval addition the same as for the Python
    // Interval.__add__() version, which just seems to do a NumPy array add of
    // lower+lower and upper+upper?
    // Can see this using the following similar code, which produces [4.3 6.4]:
    //   import numpy
    //   myarray=numpy.array([1,2])
    //   myarray2=numpy.array([3.3,4.4])
    //   print(myarray + myarray2)
    // N.B. This function is used in e.g. GridDriver::_finished_find_datum_hook()
    Interval interval_ret;
    interval_ret.lower = lower + intervalToAdd.lower;
    interval_ret.upper = upper + intervalToAdd.upper;
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
Interval Interval::combine(const Interval &otherInterval)
{
    // Returns combined interval
    Interval interval_ret = *this;
    interval_ret.assignCombine(otherInterval);
    return interval_ret;
}

//------------------------------------------------------------------------------
Interval Interval::extend(double val)
{
    // Returns interval extended by val
    Interval interval_ret = *this;
    if (val < interval_ret.lower)
    {
        interval_ret.lower = val;
    }
    else if (val > interval_ret.upper)
    {
        interval_ret.upper = val;
    }
    return interval_ret;
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

//------------------------------------------------------------------------------
Interval Interval::intersects(const Interval &otherInterval) const
{
    // Returns the intersecting range of this interval and otherInterval
    double lower_temp = std::max(otherInterval.lower, lower);
    double upper_temp = std::min(otherInterval.upper, upper);
    if (lower_temp > upper_temp)
    {
        lower_temp = std::numeric_limits<double>::quiet_NaN();
        upper_temp = std::numeric_limits<double>::quiet_NaN();
    }
    return Interval(lower_temp, upper_temp);
}


//==============================================================================
void testIntervalClass(void)
{
    // Ad-hoc Interval test function - single-step in debugger and check the
    // results

    Interval interval_1;  // N.B. Check that the intervals' internal upper and
    Interval interval_2;  // lower values default to NaN's here
    Interval interval_3;
    Interval interval_4;
    int dummy_val = 0;

    // Test toString()
    std::string interval_string;
    interval_string = Interval(std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::quiet_NaN()).toString();
    interval_string = Interval(std::numeric_limits<double>::quiet_NaN(),
                               123.0).toString();
    interval_string = Interval(2.3, 2.3).toString();
    interval_string = Interval(2.3, 5.6).toString();

    // Test == operator overload
    interval_1 = Interval(1.1, 1.2);
    interval_2 = Interval(1.1, 9.9);
    if (interval_1 == interval_2)
    {
        dummy_val++;
    }
    
    // Test != operator overload
    interval_3 = Interval(3.5, 3.6);
    if (interval_1 != interval_3)
    {
        dummy_val++;
    }
    
    // Test == operator overload
    interval_4 = Interval(1.1, 1.2);
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
    interval_2 = interval_1 - 0.5;
    
    // Test + operator overload for adding a scalar
    interval_1 = Interval(23.0, 34.0);
    interval_2 = interval_1 + 10.0;

    // Test + operator overload for adding an interval
    interval_1 = Interval(23.0, 34.0);
    interval_2 = Interval(10.6, 8.1);
    interval_3 = interval_1 + interval_2;

    // Test assignCombine()
    interval_1 = Interval(-2.34, 6.5);
    interval_2 = Interval(-3.95, 5.0);
    interval_1.assignCombine(interval_2);
    interval_3 = Interval(-2.5, 10.77);
    interval_1.assignCombine(interval_3);

    // Test combine()
    interval_1 = Interval(-2.34, 6.5);
    interval_2 = Interval(-3.95, 5.0);
    interval_3 = interval_1.combine(interval_2);
    interval_1 = Interval(-2.5, 10.77);
    interval_4 = interval_3.combine(interval_1);

    // Test extend()
    interval_1 = Interval(-3.2, 5.6);
    interval_2 = interval_1.extend(-5.1);
    interval_3 = interval_1.extend(9.8);

    // Test contains()
    interval_1 = Interval(8.0, 22.0);
    interval_2 = Interval(10.0, 20.0);
    if (interval_2.contains(interval_1))
    {
        dummy_val++;
    }
    if (interval_2.contains(interval_1, 2.0))
    {
        dummy_val++;
    }
    if (interval_2.contains(interval_1, 2.5))
    {
        dummy_val++;
    }
    if (interval_2.contains(interval_1, 1.999))
    {
        dummy_val++;
    }
    
    // Test intersects()
    interval_1 = Interval(3.2, 6.5);
    interval_2 = Interval(4.5, 7.6);
    interval_3 = interval_1.intersects(interval_2);

    dummy_val++;
}

//==============================================================================


} // namespace mpifps