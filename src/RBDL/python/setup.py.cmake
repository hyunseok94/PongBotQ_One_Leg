#!/usr/bin/env python

from distutils.core import setup
from distutils.extension import Extension
from distutils.sysconfig import get_python_lib
from Cython.Distutils import build_ext
from Cython.Build import cythonize

import os
import sys
import numpy as np

if not os.path.exists('rbdl.so'):
    print("""The setup.py script should be executed from the build directory.""")
    sys.exit(1)

lib_path = get_python_lib()[5:]

setup(
    name='rbdl',
    author='Martin Felis',
    author_email='martin@fysx.org',
    description='Python wrapper for RBDL - the Rigid Body Dynamics Library',
    license='zlib',
    version='${RBDL_VERSION_MAJOR}.${RBDL_VERSION_MINOR}.${RBDL_VERSION_PATCH}',
    url='https://rbdl.github.io/',
    cmdclass={'build_ext': build_ext},
    ext_modules=cythonize(ext_modules),
  	data_files = [(lib_path, ["rbdl.so"])],
)
