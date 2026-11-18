from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['services'],        # mirrors pps/setup.py - subpackages (services.report)
    package_dir={'': 'src'}       # resolve via __init__.py, not listed here
)

setup(**d)
