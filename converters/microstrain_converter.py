#!/usr/bin/env python3

import struct
import json

data_format="<ffffdHH"

with open(sys.argv[1], 'rb') as f:
  b = f.read()
  filesize = f.tell()

unpacker = struct.Struct(data_format)
for i in range(filesize/unpacker.size):
  
