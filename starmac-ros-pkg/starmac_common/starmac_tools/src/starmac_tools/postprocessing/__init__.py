"""
This module contains routines that are used to transform the 'raw' data recorded in bag
files to something more convenient for later analysis, plotting, etc.

For example, orientation is typically reported (and recorded) as a quaternion, but often
one would rather deal with Euler angles in degrees. The very common operation of performing this
conversion is one example of a functionality that belongs here.
"""