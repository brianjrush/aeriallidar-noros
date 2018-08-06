#!/usr/bin/env python3

from utils import gps2utc
import numpy as np
import quaternion
from transform import Transform
from math import radians

class TransformDatabase():
  def __init__(self, csv=None, npy=None):
    self.stamps = None
    self.transforms = None
    if csv is not None:
      self.read_transforms_from_csv(csv)
    elif npy is not None:
      self.read_stamps_from_npy(npy[0])
      self.read_transforms_from_npy(npy[1])

  def write_to(self, stamp_path, tf_path):
    np.save(stamp_path, self.stamps)
    np.save(tf_path, self.transforms)

  def read_transforms_from_npy(self, npy):
    self.transforms = np.load(npy)

  def read_stamps_from_npy(self, stamps):
    self.stamps = np.load(stamps)

  def read_transforms_from_csv(self, tf_file):
    ''' Data needed:
      [stamp], [x], [y], [z], [qx], [qy], [qz], [qw]
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
      qx = float(fields[4])
      qy = float(fields[5])
      qz = float(fields[6])
      qw = float(fields[7])
      stamps.append(stamp)
      tfs.append(Transform(stamp, x, y, z, qx, qy, qz, qw))
    f.close()
    self.stamps = np.array(stamps)
    self.transforms = np.array(tfs)
    #print("Successfully read %d transforms" % len(self.transforms))

  def lookup_transform(self, time):
    errors = np.abs(self.stamps - time)
    index = errors.argmin()
    err = errors.min()
    return (self.transforms[index], err)

if __name__ == '__main__':
  import sys
  if len(sys.argv) < 2:
    print("Too few arguments. Usage: %s [transform file]" % sys.argv[0], file=sys.stderr)
    sys.exit(1)
  tf_db = TransformDatabase(sys.argv[1])
  print(tf_db.transforms[0])
