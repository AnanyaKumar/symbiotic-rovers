#!/usr/bin/env python
# Template: TODO (Change this) 
from distutils.core import setup
from distutils.extension import Extension
 
setup(name="PackageName",
    ext_modules=[
        Extension("Grid", ["grid.cpp"],
        libraries = ["boost_python"])
    ])
