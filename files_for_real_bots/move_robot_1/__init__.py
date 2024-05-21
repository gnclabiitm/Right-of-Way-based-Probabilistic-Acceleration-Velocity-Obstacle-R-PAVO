# This code will be executed when the package is imported

# Import modules that will be used by multiple nodes within the package
import rospy
import numpy as np

# Define package-level variables or functions that will be used by multiple nodes
MY_PACKAGE_VERSION = '1.0'

def my_package_utils_function():
    # Code for the function goes here
    pass

# Define a module within the package
__all__ = ['obstacles_utils']
