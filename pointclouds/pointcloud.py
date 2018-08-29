#!/usr/bin/env python3

import tools.ply_formatter
from pointclouds.point import Point

class PointCloud():
  def __init__(self, points=[], ply=None):
    self.points = points
    self.stamp = -1
    if ply is not None:
      self.from_ply(ply)
  
  def from_ply(self, infile):
    self.points = tools.ply_formatter.read(infile).points

  def to_ply(self, outfile):
    tools.ply_formatter.write(self, outfile)

  def size(self):
    return len(self.points)

  def __str__(self):
    return "A pointcloud containing %d point(s)" % self.size()

  def __iter__(self):
    return iter(self.points)

  def __add__(self, other):
    if isinstance(other, self.__class__):
      self.points.extend(other.points)
    elif isinstance(other, Point):
      self.points.append(other)
    elif other is None:
      return self
    else:
      raise TypeError("Unsupported operand type(s) for +: '%s' and '%s'" % (self.__class__, type(other)))
    return self

