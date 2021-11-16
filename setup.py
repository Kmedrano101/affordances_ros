from distutils.core import setup
from setuptools import find_packages
import os

# Optional project description in README.md:

current_directory = os.path.dirname(os.path.abspath(__file__))

try:
    with open(os.path.join(current_directory, 'README.md'), encoding='utf-8') as f:
        long_description = f.read()
except Exception:
    long_description = ''

setup(
# Project name:
name='semantic-nav',

# Packages to include in the distribution:
packages=find_packages(','),

# Project version number:
version='0.0.1',

# List a license for the project, eg. MIT License
license='MIT License',

# Short description of your library:
description='',

# Long description of your library:

long_description=long_description,
long_description_content_type='text/markdown',

# Your name:
author='Kevin Medrano Ayala',

# Your email address:
author_email='kevin.ejem18@gmail.com',

# Link to your github repository or website:
url='https://github.com/Kmedrano101/tfm',

# Download Link from where the project can be downloaded from:
download_url='',

# List of keywords:
keywords=[],

# List project dependencies:
install_requires=[],

# https://pypi.org/classifiers/
classifiers=[]

)