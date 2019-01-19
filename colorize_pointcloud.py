#!/usr/bin/python3
import tools.ply_formatter as ply_formatter
from pointclouds.pointcloud import PointCloud
import sys

def colorize(point, mn, mx):
  n=(point.position[2]-mn['z'])/(mx['z']-mn['z'])
  point.color[0] = round(255*n)
  point.color[1] = round(255*(1-n))
  point.color[2] = 0

cloud_files = sys.argv[1:]

minpos={'x':0, 'y':0,'z':0}
maxpos={'x':0, 'y':0,'z':0}

full_cloud = PointCloud()
i = 1
for f in cloud_files:
  print("{}/{}".format(i, len(cloud_files)))
  i+=1
  cloud = ply_formatter.read(f)
  for point in cloud:
    if point.position[0] < minpos['x']:
      minpos['x'] = point.position[0]
    elif point.position[0] > maxpos['x']:
      maxpos['x'] = point.position[0]
    if point.position[1] < minpos['y']:
      minpos['y'] = point.position[1]
    elif point.position[1] > maxpos['y']:
      maxpos['y'] = point.position[1]
    if point.position[2] < minpos['z']:
      minpos['z'] = point.position[2]
    elif point.position[2] > maxpos['z']:
      maxpos['z'] = point.position[2]
  full_cloud+=cloud

for point in full_cloud:
  colorize(point,minpos,maxpos)

ply_formatter.write(full_cloud, "testcloud.ply")
