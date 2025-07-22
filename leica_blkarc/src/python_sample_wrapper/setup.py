from setuptools import setup

setup(
    name='blk_arc_sample_wrapper',
    version='0.9.0',
    description='The BLK ARC Module API is used to control and to communicate with BLK ARC devices.',
    url='https://shop.leica-geosystems.com/leica-blk/blk-arc',
    packages=['blk_arc_sample_wrapper'],
    extras_require={'dev': ['pytest', 'mypy', 'pytest-cov', 'mypy-protobuf']},
    author='Alex Hönger, Teng Foong Lam, Pascal Schoppmann',
    author_email='alex.hoenger@hexagon.com, tengfoong.lam@hexagon.com, pascal.schoppmann@hexagon.com',
)
