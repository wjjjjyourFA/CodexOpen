from setuptools import setup, find_packages
import os

def read_requirements(filename):
    """Read dependencies from a requirements file."""
    with open(filename) as f:
        return f.read().splitlines()
