#!/usr/bin/env python3

from pointcloud import PointCloud

class PointCloudDatabase():
  def __init__(self, clouds=[], infiles=None):
    self.clouds = clouds
    if infiles is not None:
      print("Reading points from ply files")
      self.read_plys(infiles)

  def read_plys(self, infiles):
    if isinstance(infiles,list):
      for f in infiles:
        c = PointCloud(ply=f)
        self.clouds.append(c)
    else:
      c = PointCloud(ply=f)
      self.clouds.append(c)

  def write_plys(self, prefix=""):
    count = 0
    for cloud in self.clouds:
      cloud.to_ply(prefix+count+".ply")
      count+=1

  def __iter__(self):
    return iter(self.clouds)
  def __add__(self, other):
    if isinstance(other, self.__class__):
      return PointCloudDatabase(clouds=self.clouds + other.clouds)
    elif isinstance(other, PointCloud):
      clouds = self.clouds[:]
      clouds.append(other)
      return PointCloudDatabase(clouds=clouds)
    else:
      raise TypeError("Unsupported operand type(s) for +: '%s' and '%s'" % (self.__class__, type(other)))
