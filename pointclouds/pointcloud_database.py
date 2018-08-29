#!/usr/bin/env python3

from pointclouds.pointcloud import PointCloud

class PointCloudDatabase():
  def __init__(self, clouds=[], infiles=None):
    if infiles is not None:
      print("Reading points from ply files")
      self.read_plys(infiles)
    self.clouds.extend(clouds)

  def read_plys(self, infiles):
    self.clouds = [None]*len(infiles)
    for i in range(len(infiles)):
      c = PointCloud(ply=infiles[i])
      self.clouds[i] = c

  def write_plys(self, prefix=""):
    count = 0
    for cloud in self.clouds:
      cloud.to_ply(prefix+count+".ply")
      count+=1

  def __iter__(self):
    return iter(self.clouds)
  def __add__(self, other):
    if isinstance(other, self.__class__):
      self.clouds.extend(other.clouds)
    elif isinstance(other, PointCloud):
      clouds.append(other)
    else:
      raise TypeError("Unsupported operand type(s) for +: '%s' and '%s'" % (self.__class__, type(other)))
    return self
