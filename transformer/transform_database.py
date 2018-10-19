#!/usr/bin/env python3

from tools import utils
import numpy as np
from transformer.transform import Transform
import json
from math import radians
import utm

class TransformDatabase():
  def __init__(self, f):
    self.stamps = None
    self.transforms = None
    self.read_transforms_from_json(f) 

  def read_transforms_from_json(self, jsonfile):
    # read json with dict[timestamp] = {x,y,z,qx,qy,qz,qw}
    with open(jsonfile) as f:
      data=json.load(f)

    self.transforms = np.empty(len(data.keys()), dtype=object)
    self.stamps = np.empty(len(data.keys()))
    count = 0

    for stamp, att in data.items():
      stamp = float(stamp)
      self.transforms[count] = Transform(stamp=stamp, x=att['x'], y=att['y'], z=att['z'], qx=att['qx'], qy=att['qy'], qz=att['qz'], qw=att['qw'])
      self.stamps[count] = stamp
      count += 1

  def lookup_transform(self, stamp):
    index, err = utils.get_closest_stamp(stamp, self.stamps)
    return (self.transforms[index], err)

if __name__ == '__main__':
  import sys
  if len(sys.argv) < 2:
    print("Too few arguments. Usage: %s [transform file]" % sys.argv[0], file=sys.stderr)
    sys.exit(1)
  tf_db = TransformDatabase(sys.argv[1])
  print(tf_db.transforms[0])
