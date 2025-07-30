from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['pcan_ethernet_gateway'],              # Tên package Python (phải trùng với thư mục trong src/)
    package_dir={'': 'src'}       # Tất cả module đều trong src/
)

setup(**d)
