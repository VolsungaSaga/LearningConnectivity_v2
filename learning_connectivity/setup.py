## ! DO NOT MANUALLY INVOKE THIS WITH PYTHON, USE CATKIN INSTEAD 
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
	packages=['learning_connectivity'],
	package_dir={'': 'include'},


	)

setup(**setup_args)