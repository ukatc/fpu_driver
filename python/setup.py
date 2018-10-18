#!/usr/bin/env python

from distutils.core import setup

setup(name='fpu_driver',
      version='1.3.0',
      description='MOONS FPU EtherCAN interface module',
      author='Johannes Nix',
      author_email='johannes.nix@stfc.ac.uk',
      py_modules=['FpuGridDriver', 'fpu_commands', 'wflib']
      ext_modules=[Extension('ethercanif', ['src/ethercanif.cpp'])],
     )
