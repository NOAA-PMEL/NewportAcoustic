# emulate floats v3
from design import *
import winch

name = 'floats'

def depth():
    "depth of floats (between ant and buoy)"
    d = mooring - (winch.cable() + buoyLine + floatsLine)
    # floats in water, but not in air ;-)
    if d<0: d = 0
    return d
    # tbd: take off some due to current
