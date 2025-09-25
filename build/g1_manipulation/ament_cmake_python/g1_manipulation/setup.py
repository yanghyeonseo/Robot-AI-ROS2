from setuptools import find_packages
from setuptools import setup

setup(
    name='g1_manipulation',
    version='0.0.0',
    packages=find_packages(
        include=('g1_manipulation', 'g1_manipulation.*')),
)
