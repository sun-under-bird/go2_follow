from setuptools import find_packages
from setuptools import setup

setup(
    name='uwb_aoa_pkg',
    version='0.0.0',
    packages=find_packages(
        include=('uwb_aoa_pkg', 'uwb_aoa_pkg.*')),
)
