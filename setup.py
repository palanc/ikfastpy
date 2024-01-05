from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
from setuptools import setup, find_packages

setup(name='ikfastpy',
      version='1.0.0',
      url='https://github.com/palanc/ikfastpy',
      author='palanc',
      author_email='planc509@gmail.com',
      description='IK for Franka arm',
      packages=find_packages(),
      install_requires=['numpy', 'Cython'],
      ext_modules=[Extension("ikfastpy", 
                            ["ikfastpy.pyx", 
                             "ikfast_wrapper.cpp"], language="c++", libraries=['lapack'])],
      cmdclass = {'build_ext': build_ext})

