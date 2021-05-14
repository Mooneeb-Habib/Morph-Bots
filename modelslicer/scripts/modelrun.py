#!/usr/bin/python

import ModelSlicer
import numpy
from stl import mesh

img = "cube-side-125.stl"

slce = ModelSlicer.initialize()
goal, sh = slce.startSlicer(img)

print(goal)