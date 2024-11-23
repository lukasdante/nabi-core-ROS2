from setuptools import find_packages
from setuptools import setup

setup(
    name='nabi_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('nabi_interfaces', 'nabi_interfaces.*')),
)
