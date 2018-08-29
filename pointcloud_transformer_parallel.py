#!/usr/bin/python3
import os
import json
from transformer.transform_database import TransformDatabase
from transformer.transform import Transform
import tools.ply_formatter as ply_formatter
from pointclouds.pointcloud import PointCloud 
import numpy as np
from pointclouds.point import Point
import multiprocessing

output_dir = ''
stamp_file = None
tf_file = None
tf_db = None
strip = None
min_rgb = None
sections = None
error_limit = 0.1
# transform from quanergy (North West Up) to Pixhawk (North East Down)
#                                                 w   x    y   z
static_transform = np.array([0.707,0,0.707,0])

error_counter = multiprocessing.Value('i', 0)
skipped_counter = multiprocessing.Value('i', 0)

def transform(tfs, cloud, limit=0.1):
  tf, error = tfs.lookup_transform(cloud.stamp)
  if error > limit:
    #print("OUT OF SYNC. Looking for %f, found %f. Error of %f seconds" % (cloud.stamp, tf.stamp, error))
    global error_counter
    with error_counter.get_lock():
      error_counter.value += 1
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
  # rotate by tf rotation
  #transformed = quaternion.rotate_vectors(tf.rotation, transformed)
  transformed = rotate_point(static_transform, point.position)
  transformed = rotate_point(tf.rotation, transformed)

  # then add the translation, and create a new point from this position
  transformed += tf.translation
  transformed_point = Point(transformed[0], transformed[1], transformed[2], point.color[0], point.color[1], point.color[2])
  return transformed_point

def run_file(f):
  cloud = ply_formatter.read(f, min_rgb=min_rgb, strip_min=strip)
  if sections is not None:
    for section in sections:
      if cloud.stamp >= section['start'] and cloud.stamp < section['end']:
        if section['inair']:
          break
        else:
          global skipped_counter
          with skipped_counter.get_lock():
            skipped_counter.value += 1
          return
  transformed_cloud = transform(tf_db, cloud, limit=error_limit)
  if transformed_cloud is not None and transformed_cloud.size() is not 0:
    outfile = os.path.join(output_dir, os.path.basename(f))
    ply_formatter.write(transformed_cloud, outfile)

if __name__ == '__main__':
  import sys
  import argparse

  # parse arguments
  parser = argparse.ArgumentParser(description="Process .ply files from a LiDAR and transform them using transforms stored in a json file.")
  parser.add_argument('output_dir', type=str, help='The output directory. This must be set')
  parser.add_argument('json', type=str, help='The json file containing transforms')
  parser.add_argument('clouds', metavar='cloud', type=str, nargs='+', help='ply files to transform')
  parser.add_argument('-j', metavar='jobs', type=int, help='The number of jobs to run', default=1)
  parser.add_argument('-m', '--min-rgb', metavar='VAL', type=int, help='Set a minimum rgb value (0-255) to all points (default is 0)', default=0)
  parser.add_argument('-l', '--limit', metavar='SECS', type=float, help='Set the maximum error between the timestamp on the pointcloud, and the timestamp on the transform')
  parser.add_argument('--strip', action='store_true', help='Used  with the -m option. Set whether any points below the minimum rgb value should be increased to minimum, or ignored (default is increased)')
  parser.add_argument('--sections', help='Supply a section file to help reduce computation. Any section not marked as in air will be ignored', default=None)

  args = parser.parse_args()

  total_clouds = len(args.clouds)
  jobs = args.j
  print('Transforming %d files with %d jobs' % (total_clouds, jobs))
  width = len(str(total_clouds))
  output_dir = args.output_dir
  tf_json = args.json
  tf_db = TransformDatabase(jsonfile=tf_json)
  strip = args.strip
  error_limit = args.limit
  min_rgb = args.min_rgb
  if args.sections is not None:
    with open(args.sections) as f:
      sections = json.load(f)

  pool = multiprocessing.Pool(processes=jobs)
  import time
  start_time = time.time()
  for i,_ in enumerate(pool.imap_unordered(run_file, args.clouds), 1):
    if i % 500 == 0 and i != 0:
      sys.stdout.write(" - %.2f secs\n" % (time.time() - start_time))
      start_time = time.time()
    sys.stdout.write("\r{:0>{w}}/{}: {:^{w}} successful  {:^{w}} ignored  {:^{w}} out of sync".format(i, total_clouds, i-error_counter.value-skipped_counter.value, skipped_counter.value, error_counter.value, w=width))
  sys.stdout.write('\n')
  pool.close()
  pool.join()
