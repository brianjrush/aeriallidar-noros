#!/usr/bin/python3
from transform_database import TransformDatabase
from transform import Transform
import ply_formatter as ply_formatter
from pointcloud import PointCloud
import numpy as np
from point import Point

def transform(tfs, cloud):
  tf, error = tfs.lookup_transform(cloud.stamp)
  if error > 0.25:
    print("OUT OF SYNC. Looking for %f, found %f. Error of %f seconds" % (cloud.stamp, tf.stamp, error))
    return None
  else:
    print("Looking for %f, found %f. Error of %f seconds" % (cloud.stamp, tf.stamp, error))
  transformed_cloud = transform_cloud(cloud, tf)
  return transformed_cloud

def transform_cloud(cloud, tf):
  transformed_cloud = PointCloud()
  for point in cloud:
    transformed_cloud += transform_point(point, tf)
  return transformed_cloud

def rotate_point(p, q):
  # rotate point as per voodoo magic found here: https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion
  u = q[:3] # extract q.x, q.y, and q.z
  s = q[3] # q.w
  transformed = 2. * np.dot(u, p) * u \
                + (s*s - np.dot(u, u)) * p \
                + 2. * s * np.cross(u, p)
  return transformed

def transform_point(point, tf):
  # rotate by fixed transform
  #fixed_tf = np.array([0.707, 0, -0.707, 0])
  #transformed = rotate_point(point.position, fixed_tf)

  #fixed_tf = np.array([0,0,1,0])
  #transformed = rotate_point(transformed, fixed_tf)
  fixed_tf = np.array([0,.707,0,-.707])
  transformed = rotate_point(point.position, fixed_tf)

  # rotate by tf rotation
  transformed = rotate_point(transformed, tf.rotation)

  # then add the translation, and create a new point from this position
  transformed += tf.translation
  transformed_point = Point(transformed[0], transformed[1], transformed[2], point.color[0], point.color[1], point.color[2])
  return transformed_point

if __name__ == '__main__':
  import sys
  if len(sys.argv) < 5:
    print('Not enough arguments. Usage: %s [pointcloud.ply] [stamps.npy] [tfs.npy] [output.ply]' % sys.argv[0])
    sys.exit(1)
  path = sys.argv[1]
  stamps = sys.argv[2]
  tfs = sys.argv[3]
  out_file = sys.argv[4]
  cloud = ply_formatter.read(path)
  tfs = TransformDatabase(npy=(stamps,tfs))
  transformed_cloud = transform(tfs, cloud)
  if transformed_cloud is not None:
    ply_formatter.write(transformed_cloud, out_file)
    print("Transformed file %s to %s" % (path, out_file))
  
