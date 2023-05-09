import glob 
import os 
from setuptools import setup, Extension 


from utils import C_DIRECTORY

__version__ = "0.0.1" 

os.environ["CC"] = "gcc"

c_extension: Extension = Extension(
        'control_c', 
        sources=[*glob.glob(os.path.join(C_DIRECTORY, "*.c"))], 
        include_dirs=[*glob.glob(os.path.join(C_DIRECTORY, "*.h"))], 
        extra_compile_args=['-Wall', '-Os']
        )

setup(
    name="control_c", 
    version=__version__, 
    author="Nick Richardson", 
    author_email="njkrichardson@princeton.edu", 
    description="Python bindings for C implementation of the vehicle controller", 
    ext_modules=[c_extension], 
    python_requires=">=3.9", 
)
