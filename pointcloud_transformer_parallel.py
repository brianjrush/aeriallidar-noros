#!/usr/bin/python3
import os
from transform_database import TransformDatabase
from transform import Transform
import ply_formatter as ply_formatter
from pointcloud import PointCloud
import numpy as np
from point import Point

output_dir = ''
stamp_file = None
tf_file = None
tf_db = None

def transform(tfs, cloud):
  tf, error = tfs.lookup_transform(cloud.stamp)
  if error > 0.25:
    #print("OUT OF SYNC. Looking for %f, found %f. Error of %f seconds" % (cloud.stamp, tf.stamp, error))
    return None
  #else:
  #  print("Looking for %f, found %f. Error of %f seconds" % (cloud.stamp, tf.stamp, error))
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
  fixed_tf = np.array([0.707, 0, -0.707, 0])
  transformed = rotate_point(point.position, fixed_tf)

  fixed_tf = np.array([0,0,1,0])
  transformed = rotate_point(transformed, fixed_tf)
  #fixed_tf = np.array([0,.707,0,-.707])
  #transformed = rotate_point(point.position, fixed_tf)

  # rotate by tf rotation
  transformed = rotate_point(transformed, tf.rotation)

  # then add the translation, and create a new point from this position
  transformed += tf.translation
  transformed_point = Point(transformed[0], transformed[1], transformed[2], point.color[0], point.color[1], point.color[2])
  return transformed_point

def run_file(f):
  cloud = ply_formatter.read(f, min_rgb=5)
  transformed_cloud = transform(tf_db, cloud)
  if transformed_cloud is not None and transformed_cloud.size() is not 0:
    outfile = os.path.join(output_dir, os.path.basename(f))
    ply_formatter.write(transformed_cloud, outfile)
    #print("Transformed file %s to %s" % (f, outfile))
 
if __name__ == '__main__':
  import sys
  from multiprocessing import Pool
  if len(sys.argv) < 5:
    print('Not enough arguments. Usage: %s [output_dir] [stamps.npy] [tfs.npy] [input.ply...]' % sys.argv[0])
    sys.exit(1)
  output_dir = sys.argv[1]
  stamp_file = sys.argv[2]
  tf_file = sys.argv[3]
  total_args = len(sys.argv) - 4
  tf_db = TransformDatabase(npy=(stamp_file,tf_file))

  pool = Pool(processes=8)
  run_file(sys.argv[4])
  run_file(sys.argv[5])
  run_file(sys.argv[6])
#  import time
#  start_time = time.time()
#  for i,_ in enumerate(pool.imap_unordered(run_file, sys.argv[4:]), 1):
#    if i % 500 == 0 and i != 0:
#      sys.stdout.write(" - %.2f seconds elapsed\n" % (time.time() - start_time))
#      start_time = time.time()
#    sys.stdout.write("\rTransformed %d/%d files" % (i, total_args))
#  print("Done.")
