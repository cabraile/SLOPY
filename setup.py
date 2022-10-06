import os
from setuptools import setup

def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()

setup(
    name = "slopy",
    version = "0.0.1",
    author = "Carlos André Braile Przewodowski Filho",
    author_email = "andre.prze.filho@gmail.com",
    description = ("Simple Laser Odometry in written in Python."),
    keywords = "localization slam odometry",
    url = "http://packages.python.org/an_example_pypi_project",
    packages=['slopy'],
    long_description=read('README.md'),
    classifiers=[],
)