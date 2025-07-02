# NOTE: the copy of this file in your build directory will get
# overwritten every time you recompile. To make permanent changes
# edit the version in the python/ directory instead.


import os
import os.path

from setuptools import setup, Extension
from setuptools.command.install import install
#import distutils
from setuptools import Command
try:
    from setuptools.command.build import build
    pass
except ModuleNotFoundError:
    from distutils.command.build import build
    pass

from setuptools.command.build_ext import build_ext
from sysconfig import get_config_var

package_data = {
    "ADIS16465DgPy": ["*.dpi"]
}


setup(name="ADIS16465DgPy",
      description="Data Guzzler Module for the ADIS16465 IMU",
      author="Thomas Denning",

      packages=["ADIS16465DgPy"],
      package_data=package_data)


            
