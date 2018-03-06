#!/usr/bin/env python

from distutils.core import setup

setup(name='fpu_driver',
      version='0.5.0',
      description='MOONS FPU CAN driver module',
      author='Johannes Nix',
      author_email='johannes.nix@stfc.ac.uk',
      py_modules=['FpuGridDriver', 'fpu_commands']
      ext_modules=[Extension('fpu_driver', ['src/fpu_driver.cpp'])],
     )
