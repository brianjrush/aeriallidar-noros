#!/usr/bin/env python3

import struct
import pointcloud
import point
from math import isnan

class PLYParseError(Exception):
  pass

def read(infile):
  # VERY basic parser. Assumes only vertices, does not check count, and all properties must be of the form 'property type name'
  malformed = True

  f = open(infile, 'rb')

  line = f.readline().decode().strip()
  if line == 'ply':
    # first line is always ply
    fmt = f.readline().decode().strip().split(' ')
    # fmt line can be either format ascii 1.0
    #                        format binary_little_endian 1.0
    #                        format binary_big_endian 1.0
    if len(fmt) == 3 and fmt[0] == 'format' and fmt[2] == '1.0':
      if fmt[1] == 'ascii':
        raise NotImplementedError("ASCII PLY files currently not implemented")
      elif fmt[1] == "binary_little_endian":
        endian = "<"
        malformed = False
      elif fmt[1] == "binary_big_endian":
        endian = ">"
        malformed = False

  if malformed:
    raise PLYParseError("Malformed format specification")

  elements = []
  data_indices = {}
  current_element = None
  struct_formats = []
  struct_format = endian
  i=0

  line = f.readline().decode().strip()
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
      if fields[1] == 'float':
        struct_format += 'f'
      elif fields[1] == 'uchar':
        struct_format += 'B'
      elif fields[1] == 'char':
        struct_format += 'c'
      elif fields[1] == 'short':
        struct_format += 'h'
      elif fields[1] == 'ushort':
        struct_format += 'H'
      elif fields[1] == 'int':
        struct_format += 'i'
      elif fields[1] == 'uint':
        struct_format += 'I'
      elif fields[1] == 'double':
        struct_format += 'd'
      else:
        raise PLYParseError("Unable to parse line '%s'" % line)
      data_indices[fields[2]] = i
      i+=1
    line = f.readline().decode().strip()
  data_indices['type'] = current_element
  data_indices['format'] = struct_format
  if current_element is not None:
    elements.append(data_indices)

  # skip data before vertex elements we don't care about
  offset = 0
  has_timestamp = False
  end_of_header = f.tell()
  for element in elements:
    if element['type'] == 'vertex':
      vertices = element
      vertices['seek_position'] = end_of_header + offset
      vertex_parser = struct.Struct(vertices['format'])
    elif element['type'] == 'timestamp':
      has_timestamp = True
      timestamp = element
      timestamp['seek_position'] = end_of_header + offset
      timestamp_parser = struct.Struct(timestamp['format'])
    offset += struct.calcsize(element['format']) * element['length']

  f.seek(vertices['seek_position'])
  stop = vertices['seek_position'] + (vertex_parser.size * vertices['length'])
  data = f.read(vertex_parser.size)
  count = 1
  cloud = pointcloud.PointCloud()
  while f.tell() <= stop and len(data) == vertex_parser.size:
    fields = vertex_parser.unpack(data)
    count+=1
    try:
      x = fields[vertices['x']]
      y = fields[vertices['y']]
      z = fields[vertices['z']]
      r = fields[vertices['red']]
      g = fields[vertices['green']]
      b = fields[vertices['blue']]
      if not isnan(x) and not isnan(y) and not isnan(z):
        cloud += point.Point(x,y,z,r,g,b)
        
    except KeyError:
      raise PLYParseError("Unable to parse binary data with format string %s" % struct_format)

    data = f.read(vertex_parser.size)
  if has_timestamp:
    # add timestamp to cloud
    f.seek(timestamp['seek_position'])
    
    fields = timestamp_parser.unpack(f.read(timestamp_parser.size))
    cloud.stamp = fields[0]
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
