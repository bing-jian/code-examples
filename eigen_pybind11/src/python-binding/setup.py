import os, sys

from distutils.core import setup, Extension
from distutils import sysconfig

cpp_args = ['-std=c++11', '-stdlib=libc++', '-mmacosx-version-min=10.7']

ext_modules = [
    Extension(
        'wrapper',
        [
            '../funcs.cc', '../math_utils.cc', '../geometry_utils.cc',
            '../distance_transform_utils.cc', 'pybind11_interface.cc'
        ],
        include_dirs=['pybind11/include'],
        language='c++',
        extra_compile_args=cpp_args,
    ),
]

setup(
    name='wrapper',
    version='0.0.1',
    author='Bing Jian',
    author_email='bing.jian@gmail.com',
    description='Example for using Eigen and pybind11.',
    ext_modules=ext_modules,
)
