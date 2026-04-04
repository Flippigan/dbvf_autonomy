from setuptools import find_packages
from setuptools import setup

setup(
    name='dbvf_autonomy',
    version='0.0.1',
    packages=find_packages(
        include=('dbvf_autonomy', 'dbvf_autonomy.*')),
)
