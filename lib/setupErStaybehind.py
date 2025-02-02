#!/usr/bin/env python
"""
This file belong to https://github.com/snolfi/evorobotpy
Author: Stefano Nolfi, stefano.nolfi@istc.cnr.it

setupErStaybehind.py, python wrapper for predprey.cpp

This file is part of the python module ErStaybehind.so that include the following files:
staybehind.cpp, staybehind.h, robot-env.cpp, robot-env.h, utilities.cpp, utilities.h, ErStaybehind.pxd, ErStaybehind.pyx and setupErStaybehind.py
And can be compiled with cython and installed with the commands: cd ./evorobotpy/lib; python3 setupErStaybehind.py build_ext –inplace; cp ErStaybehind*.so ../bin
"""

from distutils.core import setup
from distutils.extension import Extension
from Cython.Build import cythonize
from Cython.Distutils import build_ext
import numpy

# linux
include_gsl_dir = "/usr/include/gsl"
lib_gsl_dir = "/usr/lib/x86_64-linux-gnu"

# mac os
# include_gsl_dir = "/usr/local/Cellar/gsl/2.4/lib"
# lib_gsl_dir = "/usr/local/Cellar/gsl/2.4/include"

setup(
    cmdclass = {'build_ext': build_ext},
    ext_modules = [Extension("ErStaybehind",
                             sources=["ErStaybehind.pyx", "Boid.cpp","Vector2D.cpp"],
                             language="c++",
                             include_dirs=[numpy.get_include(), include_gsl_dir],
			     libraries=["gsl", "gslcblas"],
			     library_dirs=[lib_gsl_dir])],
)



