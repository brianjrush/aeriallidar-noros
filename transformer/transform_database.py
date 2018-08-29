#!/usr/bin/env python3

from tools.utils import gps2utc
import numpy as np
from transformer.transform import Transform
import json
from math import radians
import utm

class TransformDatabase():
  def __init__(self, csv=None, jsonfile=None):
    self.stamps = None
    self.transforms = None
    if csv is not None:
      self.read_transforms_from_csv(csv)
    if jsonfile is not None:
      self.read_transforms_from_json(jsonfile) 

  def read_transforms_from_json(self, jsonfile):
    with open(jsonfile) as f:
      data=json.load(f)
    msg_types = ['NKQ1', 'NKF1'] #['NKF1', 'NKQ1', 'BARO']
    lengths = [len(data[fmt].keys()) for fmt in msg_types]
    slowest_msg_idx = np.argmin(lengths)
    slowest_msg = msg_types[slowest_msg_idx]

    stamp_arrays = {}
    for msg in msg_types:
      stamp_arrays[msg] = np.array([float(stamp) for stamp in data[msg].keys()])

    self.transforms = np.empty(lengths[slowest_msg_idx], dtype=object)
    self.stamps = np.empty(lengths[slowest_msg_idx])
    count = 0

    for stamp in data[slowest_msg].keys():
      stamp = float(stamp)
      idx, err = self.get_closest_stamp(stamp, stamp_arrays['NKF1'])
      nkf1 = data['NKF1'][str(stamp_arrays['NKF1'][idx])]
      #idx, err = self.get_closest_stamp(stamp, stamp_arrays['BARO'])
      #baro = data['BARO'][str(stamp_arrays['BARO'][idx])]
      idx, err = self.get_closest_stamp(stamp, stamp_arrays['NKQ1'])
      nkq1 = data['NKQ1'][str(stamp_arrays['NKQ1'][idx])]
      #idx, err = self.get_closest_stamp(stamp, stamp_arrays['POS'])
      #pos = data['POS'][str(stamp_arrays['POS'][idx])]
      # ['NKQ1']['Q1'] is 'w' ['Q2']->['Q4'] are XYZ
      ## https://github.com/ArduPilot/ardupilot/blob/20d22f3629d81ccd69282c38903b46434c980ef5/libraries/AP_Math/quaternion.h
      
      #utminfo = utm.from_latlon(pos['Lat'], pos['Lng'])
      self.transforms[count] = Transform(stamp=stamp, x=nkf1['PN'], y=nkf1['PE'], z=nkf1['PD'], qx=nkq1['Q2'], qy=nkq1['Q3'], qz=nkq1['Q4'], qw=nkq1['Q1'])
      self.stamps[count] = stamp
      count += 1

  def read_transforms_from_csv(self, tf_file):
    ''' Data needed:
      [stamp], [x], [y], [z], [qw], [qx], [qy], [qz]
    '''
    #print("Reading transforms from %s" % tf_file)
    tfs = []
    stamps = []
    f = open(tf_file)
    for line in f:
      fields = line.strip().split(',')
      stamp = float(fields[0])
      x = float(fields[1])
      y = float(fields[2])
      z = float(fields[3])
      qw = float(fields[4])
      qx = float(fields[5])
      qy = float(fields[6])
      qz = float(fields[7])
      stamps.append(stamp)
      tfs.append(Transform(stamp=stamp, x=x, y=y, z=z, qw=qw, qx=qx, qy=qy, qz=qz))
    f.close()
    self.stamps = np.array(stamps)
    self.transforms = np.array(tfs)
    #print("Successfully read %d transforms" % len(self.transforms))

  def get_closest_stamp(self, desired_stamp, list_of_stamps):
    errors = np.abs(list_of_stamps - desired_stamp)
    index = errors.argmin()
    err = errors.min()
    return (index, err)

  def lookup_transform(self, stamp):
    index, err = self.get_closest_stamp(stamp, self.stamps)
    return (self.transforms[index], err)

if __name__ == '__main__':
  import sys
  if len(sys.argv) < 2:
    print("Too few arguments. Usage: %s [transform file]" % sys.argv[0], file=sys.stderr)
    sys.exit(1)
  tf_db = TransformDatabase(sys.argv[1])
  print(tf_db.transforms[0])
