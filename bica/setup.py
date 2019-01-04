from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['bica'],
    scripts=['src/bica/bica.py'],
    package_dir={'': 'src'}
)

setup(**d)
