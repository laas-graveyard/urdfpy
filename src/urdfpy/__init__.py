"""
This package provide an URDF parser written in Python.

It is divided into interfaces and parser.

Typically, the user should call parser.parse, parser.parseString or
parser.parseFromParameterServer to retrieve the URDF tree.

You may then check out the interface module to know more about the
returned structure attributes.
"""

from __future__ import print_function

from interface import *
from parser import *
