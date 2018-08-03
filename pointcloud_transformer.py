#!/usr/bin/env python3

import sys
import os
import numpy as np
from point import Point
from pointcloud import PointCloud
import ply_formatter
from transform_database import TransformDatabase
from transform import Transform

class PointCloudTransformer():
  def __init__(self, insfile):
    self.tfs = TransformDatabase(insfile)

  def transform_all(self, ply_files, output_dir):
    count = 1
    for path in ply_files:
      cloud = ply_formatter.read(path)
      transformed_cloud = self.transform(cloud)
      if transformed_cloud is not None:
        path = os.path.join(output_dir, "%d.ply" % count)
        ply_formatter.write(transformed_cloud, path)
        print("Transformed file %d/%d" % (count, len(ply_files)))
      count += 1

  def transform(self, cloud):
    tf, error = self.tfs.lookup_transform(cloud.stamp)
    if error > 0.25:
      print("OUT OF SYNC. Looking for %f, found %f. Error of %f seconds" % (cloud.stamp, tf.stamp, error))
      return None
    else:
      print("Looking for %f, found %f. Error of %f seconds" % (cloud.stamp, tf.stamp, error))
    transformed_cloud = PointCloudTransformer.transform_cloud(cloud, tf)
    return transformed_cloud

  @staticmethod
  def transform_cloud(cloud, tf):
    transformed_cloud = PointCloud()
    for point in cloud:
      transformed_cloud += PointCloudTransformer.transform_point(point, tf)
    return transformed_cloud

  @staticmethod
  def rotate_point(p, q):
    u = q[:3] # extract q.x, q.y, and q.z
    s = q[3] # q.w
    transformed = 2. * np.dot(u, p) * u \
                  + (s*s - np.dot(u, u)) * p \
                  + 2. * s * np.cross(u, p) 
    return transformed



  @staticmethod
  def transform_point(point, tf):
    # first rotate point as per voodoo magic found here: https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion

    # rotate by fixed transform
    fixed_tf = np.array([0.707, 0, -0.707, 0])
    transformed = PointCloudTransformer.rotate_point(point.position, fixed_tf)

    fixed_tf = np.array([0,0,1,0])
    transformed = PointCloudTransformer.rotate_point(transformed, fixed_tf)

    # rotate by tf rotation
    transformed = PointCloudTransformer.rotate_point(transformed, tf.rotation)

    # then add the translation, and create a new point from this position
    transformed += tf.translation
    transformed_point = Point(transformed[0], transformed[1], transformed[2], point.color[0], point.color[1], point.color[2])
    return transformed_point
    

if __name__ == '__main__':
  if len(sys.argv) < 4:
    print("Too few arguments. Usage: %s [pointcloud dir] [ins file] [output dir]" % sys.argv[0], file=sys.stderr)
    sys.exit(1)
  import glob
  path = os.path.join(sys.argv[1], "*.ply")
  files=glob.glob(path)
  pcGenerator = PointCloudTransformer(sys.argv[2])
  pcGenerator.transform_all(files, sys.argv[3])
