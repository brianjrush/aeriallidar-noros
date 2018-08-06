#!/usr/bin/env python3

import sys
sys.path.append('..')
from transform_database import TransformDatabase

if __name__ == '__main__':
  if len(sys.argv) < 4:
    print("Too few arguments. Usage: %s [transforms.csv] [output-stamp.npy] [output-tf.npy]" % sys.argv[0])
    sys.exit(1)
  tf_db = TransformDatabase(csv=sys.argv[1])
  tf_db.write_to(sys.argv[2], sys.argv[3])

