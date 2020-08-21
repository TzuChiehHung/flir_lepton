# from distutils.core import setup
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['flir_lepton_purethermal2'],
    package_dir={'': 'src'}
)

setup(**d)