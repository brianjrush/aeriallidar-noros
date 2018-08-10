#!/usr/bin/env python3

import numpy as np

class Transform():
  def __init__(self, *,stamp, x, y, z, qx, qy, qz, qw):
    self.stamp = stamp
    self.translation = np.array([x,y,z])
    self.rotation = np.array([qw,qx,qy,qz])

  def __str__(self):
    return ("""Transform recorded at %f
     Translation (x, y, z): %s
     Rotation (w, x, y, z): %s""" % (self.stamp, self.translation, self.rotation))

