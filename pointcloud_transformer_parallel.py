#!/usr/bin/python3
import os
import quaternion
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
# transform from quanergy (North West Up) to Pixhawk (North East Down)
#                                                 w   x    y   z
static_transform = np.array([0.707,0,0.707,0])

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
  transformed_cloud = PointCloud(points=[])
  for point in cloud:
    transformed_cloud += transform_point(point, tf)
  return transformed_cloud

def rotate_point(q, p):
  # rotate point as per voodoo magic found here: https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion
  #print(q, file=sys.stderr)
  u = q[1:4] # extract q.x, q.y, and q.z
  s = q[0] # q.w
  transformed = 2. * np.dot(u, p) * u \
                + (s*s - np.dot(u, u)) * p \
                + 2. * s * np.cross(u, p)
  return transformed

def transform_point(point, tf):
  # rotate by fixed transform
  #fixed_tf = quaternion.from_float_array([0,-0.707, 0,0.707])
  #transformed = quaternion.rotate_vectors(fixed_tf,point.position)

  #fixed_tf = quaternion.from_float_array([0,1,0,0])
  #transformed = quaternion.rotate_vectors(fixed_tf,transformed)
  #fixed_tf = np.array([0,.707,0,-.707])
  #transformed = rotate_point(point.position, fixed_tf)

  # rotate by tf rotation
  #transformed = quaternion.rotate_vectors(tf.rotation, transformed)
  transformed = rotate_point(static_transform, point.position)
  transformed = rotate_point(tf.rotation, transformed)

  # then add the translation, and create a new point from this position
  transformed += tf.translation
  transformed_point = Point(transformed[0], transformed[1], transformed[2], point.color[0], point.color[1], point.color[2])
  return transformed_point

def run_file(f):
  cloud = ply_formatter.read(f, min_rgb=5)
  transformed_cloud = transform(tf_db, cloud)
  #print(transformed_cloud, file=log)
  if transformed_cloud is not None and transformed_cloud.size() is not 0:
    outfile = os.path.join(output_dir, os.path.basename(f))
    ply_formatter.write(transformed_cloud, outfile)
    #print("Transformed file %s to %s" % (f, outfile))
 
if __name__ == '__main__':
  import sys
  from multiprocessing import Pool
  if len(sys.argv) < 4:
    print('Not enough arguments. Usage: %s [output_dir] [tfs.csv] [input.ply...]' % sys.argv[0])
    sys.exit(1)
  output_dir = sys.argv[1]
  #stamp_file = sys.argv[2]
  tf_csv = sys.argv[2]
  total_args = len(sys.argv) - 4
  tf_db = TransformDatabase(csv=tf_csv)

  pool = Pool(processes=8)
#  run_file(sys.argv[4])
#  run_file(sys.argv[5])
#  run_file(sys.argv[6])
  import time
  start_time = time.time()
  for i,_ in enumerate(pool.imap_unordered(run_file, sys.argv[4:]), 1):
    if i % 500 == 0 and i != 0:
      sys.stdout.write(" - %.2f seconds elapsed\n" % (time.time() - start_time))
      start_time = time.time()
    sys.stdout.write("\rTransformed %d/%d files" % (i, total_args))
  print("Done.")
