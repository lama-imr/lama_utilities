# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
from Cython.Build import cythonize

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['crossing_detector'],
    package_dir={'': 'src'})
setup_args['ext_modules'] = cythonize(
    'src/hello.pyx',
    language="c++"),


setup(**setup_args)
