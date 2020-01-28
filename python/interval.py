#!/usr/bin/python

from types import StringType, IntType, ListType, FloatType

from ast import literal_eval
import numpy
from numpy import array, ndarray, nan, Inf, isnan

class Interval:
    def __init__(self, *source):
        self.strformat = "%.3f"
        if len(source) == 0:
            source = nan
        elif len(source) == 1:
            source = source[0]
        else:
            assert(len(source)==2), "Interval: must be created with scalar or 2-tuple"
            source = list(source)
            source.sort()
        if (type(source) == numpy.float) or (type(source) == numpy.float64):
            source = float(source)
        if isinstance(source, Interval):
            iv = array(source.iv)
        elif type(source) == ndarray:
            assert(len(source) == 2), "Interval: must be created with scalar or 2-tuple"
            iv = source.copy()
        elif type(source) == IntType:
            iv = array([source, source],dtype=float)
        elif type(source) == FloatType:
            iv = array([source, source])
        elif type(source) == ListType:
            assert(len(source) == 2), "Interval: must be created with scalar or 2-tuple"
            iv = array(source, dtype=float)
        elif type(source) == StringType:
            l = literal_eval(source)
            assert(len(source)==2), "Interval: must be created with scalar or 2-tuple"
            iv = array(source, dtype=float)
        else:
            raise RuntimeError("Passed: value %r, type %s - "
                               "unsupported data type for interval" % (source, type(source)))

        self.iv = iv

    def copy(self):
        return Interval(self.iv)

    def as_scalar(self, tolerance=1.0e-6):
        assert(tolerance >= 0), "Interval:as_scalar: tolerance must be a positive value"
        assert(self.iv[0] == self.iv[1]), "Interval:as_scalar: Values %.3f %.3f not equal" % (self.iv[0], self.iv[1])
        #if abs(self.iv[1] - self.iv[0]) > tolerance:
        #    print("\n*** Interval:as_scalar: Values %.3f %.3f not equal. First value will be returned. ***\n" % \
        #          (self.iv[0], self.iv[1]))
        return self.iv[0]

    def __str__(self):
        if isnan(self.iv[0]) and isnan(self.iv[1]):
            return "[]"
        elif self.iv[0] == self.iv[1]:
            return self.strformat % self.iv[0]
        else:
            f = self.strformat
            return ("[" + f + ", " + f + "]") % (self.iv[0], self.iv[1])

    def __repr__(self):
        return repr(list(self.iv))


    def min(self):
        # Return minimum (or most negative) value
        return self.iv[0]

    def max(self):
        # Return maxmium (or most positive) value
        return self.iv[1]

    def min_abs(self):
        # Return value closest to 0.0. Useful when an interval spans zero.
        if abs(self.iv[0]) < abs(self.iv[1]):
           return self.iv[0]
        else:
           return self.iv[1]

    def max_abs(self):
        # Return value furthest from 0.0. Useful when an interval spans zero.
        if abs(self.iv[0]) > abs(self.iv[1]):
           return self.iv[0]
        else:
           return self.iv[1]

    def __add__(self, b):
        if not isinstance(b, Interval):
            b = Interval(b)
        iv = b.iv + self.iv
        return Interval(iv)

    def __sub__(self, b):
        if not isinstance(b, Interval):
            b = Interval(b)
        iv = self.iv - b.iv
        return Interval(iv)


    def __radd__(self, b):
        if not isinstance(b, Interval):
            b = Interval(b)
        iv = b.iv + self.iv
        return Interval(iv)

    def __rsub__(self, b):
        if not isinstance(b, Interval):
            b = Interval(b)
        iv = b.iv - self.iv
        return Interval(iv)

    def __addi__(self, b):
        if not isinstance(b, Interval):
            b = Interval(b)
        iv = b.iv
        self.iv = self.iv + iv

    def __subi__(self, b):
        if not isinstance(b, Interval):
            b = Interval(b)
        iv = b.iv

        self.iv = self.iv - iv

    def assignCombine(self, b):
        """mutating combine intervals and return result."""
        if not isinstance(b, Interval):
            b = Interval(b)

        iv = b.iv
        self.iv[0] = min(self.iv[0],iv[0])
        self.iv[1] = max(self.iv[1], iv[1])

        return self

    def combine(self,b):
        """immutable combining interval, returning result and
        not changing object."""
        iv = self.copy()
        return iv.assignCombine(b)

    def assignExtend(self, x):
        """mutating extend interval with scalar, and return result."""
        if x < self.iv[0]:
            self.iv[0] = x
        elif x> self.iv[1]:
            self.iv[1] = x

        return self


    def extend(self, x):
        """immutable extend interval, returning result
        but not changing object."""
        iv = self.copy()
        return iv.assignExtend(x)

    def contains(self, b, tolerance=0):
        assert(tolerance >= 0), "Interval:contains: tolerance must be a positive value"
        if not isinstance(b, Interval):
            b = Interval(b)
        iv = b.iv
        if ((self.iv[0] - tolerance) <= iv[0]) and ((self.iv[1] + tolerance) >= iv[1]):
            return True
        else:
            return False


    def intersects(self, b):
        """returns interval which contains all points
        which are contained both in self and b.
        """
        if not isinstance(b, Interval):
            b = Interval(b)
        iv = b.iv

        mi = max(iv[0], self.iv[0])
        ma = min(iv[1], self.iv[1])
        if mi > ma:
            mi = nan
            ma = nan
        return Interval(mi,ma)

if __name__ == "__main__":
    fred = Interval(1.0, 2.0)
    jim = Interval(1.5, 2.5)
    print("fred:", fred)
    print("jim:", jim)
    print("fred+jim", fred+jim)
    print("fred.combine(jim):", fred.combine(jim))
    print("fred.intersects(jim):", fred.intersects(jim))

    single = Interval(42.0)
    print("single:", single, "single.as_scalar:", single.as_scalar())
    print("jim:", jim, "jim.as_scalar:", jim.as_scalar())

