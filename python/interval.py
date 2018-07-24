#!/usr/bin/python

from types import StringType, IntType, ListType, FloatType

from ast import literal_eval
from numpy import array, ndarray, nan, Inf, isnan

class Interval:
    def __init__(self, *source):
        if len(source) == 0:
            source = nan
        elif len(source) == 1:
            source = source[0]
        else:
            assert(len(source)==2)
            source = list(source)
            source.sort()

        if isinstance(source, Interval):
            iv = array(source.iv)
        elif type(source) == ndarray:
            assert(len(source) == 2)
            iv = source.copy()
        elif type(source) == IntType:
            iv = array([source, source],dtype=float)
        elif type(source) == FloatType:
            iv = array([source, source])
        elif type(source) == ListType:
            assert(len(source) == 2)
            iv = array(source, dtype=float)
        elif type(source) == StringType:
            l = literal_eval(source)
            assert(len(source)==2)
            iv = array(source, dtype=float)
        else:
            raise RunTimeException("unsupported data type for interval")
        
        self.iv = iv

    def __str__(self):
        if isnan(self.iv[0]) and isnan(self.iv[1]):
            return "[]"
        elif self.iv[0] == self.iv[1]:
            return "%g" % self.iv[0]
        else:
            return str(list(self.iv))
        
    def __repr__(self):
        return repr(list(self.iv))

    def copy(self):
        return Interval(self.iv)

    def min(self):
        return self.iv[0]

    def max(self):
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

    def combine(self, b):
        """mutating combine intervals and return result."""
        if not isinstance(b, Interval):
            b = Interval(b)

        iv = b.iv
        self.iv[0] = min(self.iv[0],iv[0])
        self.iv[1] = max(self.iv[1], iv[1])

        return self
        
    def extend(self, x):
        """mutating extend interval with scalar, and return result."""
        if x < self.iv[0]:
            self.iv[0] = x
        elif x> self.iv[1]:
            self.iv[1] = x
            
        return self
            

    def iextend(self, x):
        """immutable extend interval, returning result 
        but not changing object."""
        iv = self.copy()
        return iv.extend(x)

    def icombine(self,b):
        """immutable combining interval, returning result and
        not changing object."""
        iv = self.copy()
        return iv.combine(b)
    
    def contains(self, b, tolerance=0):
        assert(tolerance >= 0)
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
    
