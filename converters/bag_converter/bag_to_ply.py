#!/usr/bin/env python2 

import sys
import tools.ply_formatter
from pointclouds.pointcloud import PointCloud
from pointclouds.point import Point
import sensor_msgs.point_cloud2 as pc2
import rosbag
import struct
from math import isnan, pow

def parse_msg(msg):
  points = pc2.read_points(msg, skip_nans=True, field_names=('x','y','z','intensity'))
  cloud = PointCloud()
  for point in points:
    #cloud += Point(point[0], point[1], point[2], point[3], point[3], point[3])
    ## color based on position
    cloud += Point(point[0], point[1], point[2], point[0] % 256, point[1] % 256, point[2] % 256)
  cloud.stamp = msg.header.stamp.to_sec()
  return cloud
  
  

def parse_bag(path, outdir):
  bag = rosbag.Bag(path)
  print("Opened bag")
  size = bag.get_message_count(topic_filters="/Sensor/points")
  count = 1
  for topic, msg, t in bag.read_messages(topics=['/Sensor/points']):
    cloud = parse_msg(msg)
    outfile = "%s/%d.ply" % (outdir, count)
    ply_formatter.write(cloud, outfile)
    print("Wrote %d/%d PLY files" % (count, size))
    count+=1
  bag.close()

if __name__ == '__main__':
  if len(sys.argv) < 3:
    print("Too few arguments. Usage: %s [bag] [ply output dir]"  % sys.argv[0])
    sys.exit(1)

  parse_bag(sys.argv[1], sys.argv[2])
