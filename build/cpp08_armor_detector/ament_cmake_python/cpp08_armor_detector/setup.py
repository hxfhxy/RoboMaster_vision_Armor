from setuptools import find_packages
from setuptools import setup

setup(
    name='cpp08_armor_detector',
    version='0.0.0',
    packages=find_packages(
        include=('cpp08_armor_detector', 'cpp08_armor_detector.*')),
)
