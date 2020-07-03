#!/usr/bin/env python
"""Put Scoot related libraries here.

Any ROS module imports here will break namespace.py's ability to modify
namespaces.
"""
from . import Scoot
from . import Driver
from . import repl
from behaviors import *
