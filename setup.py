import glob 
import os 
import setuptools 
from typing import Sequence

from pybind11.setup_helpers import Pybind11Extension 

__version__ = "0.0.1" 

extension_modules: Sequence[Pybind11Extension] = [
    Pybind11Extension(
        "control_c", 
        sorted(glob.glob("c_src/*.cpp")), 
        include_dirs=["./include"], 
        ), 

]

setuptools.setup(
    name="control_c", 
    version=__version__, 
    author="Nick Richardson", 
    author_email="njkrichardson@princeton.edu", 
    description="CPython bindings for vehicle controller", 
    ext_modules=extension_modules, 
    python_requires=">=3.9", 
)
