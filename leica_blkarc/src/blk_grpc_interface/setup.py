from setuptools import setup

setup(
    name='blk_arc_grpc',
    version='0.9.0',
    packages=['blk_arc_grpc'],
    install_requires=['grpcio==1.43.0', 'grpcio-tools==1.38.1', 'protobuf==3.19.4'],
    extras_require={'dev': ['pytest', 'mypy', 'pytest-cov', 'mypy-protobuf']},
)
