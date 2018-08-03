#!/usr/bin/env python3

import numpy as np

class Transform():
  def __init__(self, stamp, x, y, z, qx, qy, qz, qw):
    self.stamp = stamp
    self.translation = np.array([x,y,z])
    self.rotation = np.array([qx,qy,qz,qw])

  def __str__(self):
    return ("""Transform recorded at %f
     Translation (x, y, z): %s
     Rotation (x, y, z, w): %s""" % (self.stamp, self.translation, self.rotation))

