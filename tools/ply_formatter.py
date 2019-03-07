#!/usr/bin/env python3

import struct
import pointclouds.pointcloud as pointcloud
from math import isnan
import os

def read(infile, min_rgb=0, strip_min=False, use_filename=False):
  malformed = True

  ply_types = {'float' : 'f',
               'uchar' : 'B',
               'char'  : 'c',
               'short' : 'h',
               'ushort': 'H',
               'int'   : 'i',
               'uint'  : 'I',
               'double': 'd'};

  valid_fmt = {'binary_little_endian' : '<',
               'binary_big_endian'    : '>'}
  #            ascii unsupported

  with open(infile, 'rb') as f:
    lines = f.readlines()

  if b'ply' not in lines[0]:
    raise ValueError("Not a valid PLY file (1st line is not 'ply'")
  try:
    endian = valid_fmt[lines[1].decode().split(' ')[1]] # lines[1] should be 'format binary_little_endian|binary_big_endian 1.0'
  except KeyError as e:
    print('Invalid or unsupported format type %s' % e)
  
  line_number = 2
  line = lines[line_number]

  elements = []
  data_indices = {}
  current_element = None
  struct_formats = []
  struct_format = endian
  i=0

  line = lines[line_number].decode().strip()
  line_number+=1
  while line != 'end_header':
    fields = line.split(' ')
    if fields[0] == 'element' and len(fields) == 3:
      data_indices['type'] = current_element
      data_indices['format'] = struct_format
      if current_element is not None:
        elements.append(data_indices)
      data_indices = {}
      data_indices['length'] = int(fields[2])
      struct_format = endian
      current_element = fields[1]
    elif fields[0] == 'property' and len(fields) == 3:
      struct_format += ply_types[fields[1]]
      data_indices[fields[2]] = i
      i+=1
    line = lines[line_number].decode().strip()
    line_number+=1
  data_indices['type'] = current_element
  data_indices['format'] = struct_format
  if current_element is not None:
    elements.append(data_indices)

  # skip data before vertex elements we don't care about
  bin_data = b''.join(lines[line_number:])
  offset = 0
  has_timestamp = False
  for element in elements:
    if element['type'] == 'vertex':
      vertices = element
      vertices['seek_position'] = offset
      vertex_parser = struct.Struct(vertices['format'])
    elif element['type'] == 'timestamp':
      has_timestamp = True
      timestamp = element
      timestamp['seek_position'] = offset
      timestamp_parser = struct.Struct(timestamp['format'])
    offset += struct.calcsize(element['format']) * element['length']

  start = vertices['seek_position']
  stop = vertices['seek_position'] + (vertex_parser.size * vertices['length'])
  current = start
  data = bin_data[start:start+vertex_parser.size]
  count = 0
  points = [None] * (vertices['length'] // 10)
  while current <= stop and len(data) == vertex_parser.size:
    fields = vertex_parser.unpack(data)
    try:
      x = fields[vertices['x']]
      y = fields[vertices['y']]
      z = fields[vertices['z']]
      r = max(fields[vertices['red']], min_rgb)
      g = max(fields[vertices['green']], min_rgb)
      b = max(fields[vertices['blue']], min_rgb)
      if not strip_min or not (r == min_rgb or g == min_rgb or b == min_rgb):
        if not isnan(x) and not isnan(y) and not isnan(z):
          if count >= len(points):
            if vertices['length'] // 10 == 0:
              points.extend([None] * vertices['length'])
            else:
              points.extend([None] * (vertices['length'] // 10))
          points[count] = pointcloud.Point(x,y,z,r,g,b)
          count += 1
    except KeyError:
      raise PLYParseError("Unable to parse binary data with format string %s" % struct_format)
    current += vertex_parser.size
    data = bin_data[current:current+vertex_parser.size]
  points = points[:count]
  cloud = pointcloud.PointCloud(points=points)
  if has_timestamp:
    # add timestamp to cloud
    start = timestamp['seek_position']
    fields = timestamp_parser.unpack(bin_data[start:start+timestamp_parser.size])
    cloud.stamp = fields[0]
  elif use_filename:
    cloud.stamp = int(os.path.basename(infile).split('_')[0])/1000000
  return cloud

def pack_point(point):
  return struct.pack("<fffBBB",
                     point.position[0],
                     point.position[1],
                     point.position[2],
                     point.color[0],
                     point.color[1],
                     point.color[2])

def write(cloud, outfile):
  header = (b"ply\n"
            b"format binary_little_endian 1.0\n"
            b"element vertex %d\n"
            b"property float x\n"
            b"property float y\n"
            b"property float z\n"
            b"property uchar red\n"
            b"property uchar green\n"
            b"property uchar blue\n") % cloud.size()
  if cloud.stamp != -1:
    header += (b"element timestamp 1\n"
              b"property double stamp\n")
  header += b"end_header\n" 
  f = open(outfile, 'wb')
  f.write(header)
  for point in cloud:
    f.write(pack_point(point))
  if cloud.stamp != -1:
    f.write(struct.pack("<d", cloud.stamp))
  f.close()

if __name__ == '__main__':
  cloud=read('/home/e4e/aeriallidar-noros/transformer/1549055576168992_05587.ply')
