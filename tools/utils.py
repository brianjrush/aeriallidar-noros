#!/usr/bin/env python3

def gps2utc(tow, wk):
  gps_sec = tow + wk * 7. * 24. * 60. * 60.
  return gps_sec + 315964800. - 18. # GPS2UNIX offset - LEAP sec

import numpy as np
def get_closest_stamp(desired_stamp, list_of_stamps):
  errors = np.abs(list_of_stamps - desired_stamp)
  index = errors.argmin()
  err = errors.min()
  return (index, err)
