# setup.py
from setuptools import setup, Extension
import numpy as np

ballistic_module = Extension(
    'ballistic',
    sources=['ballistic_wrapper.c', 'SolveTrajectory.c'],
    include_dirs=[np.get_include()],
    libraries=['m']
)

setup(name='ballistic', ext_modules=[ballistic_module])