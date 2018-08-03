#!/usr/bin/env python3

import numpy as np

class Point():
  def __init__(self, x, y, z, r, g, b):
    self.position = np.array([x,y,z])
    self.color = np.array([r,g,b])

  def __str__(self):
    return """Point
      Position (x, y, z): %s
      Color    (r, g, b): %s""" % (self.position, self.color)
