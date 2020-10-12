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

#include "Interval.h"

namespace mpifps
{

//------------------------------------------------------------------------------
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
bool Interval::contains(const Interval &otherInterval, double tolerance)
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

} // namespace mpifps