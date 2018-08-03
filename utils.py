#!/usr/bin/env python3

def gps2utc(tow, wk):
  gps_sec = tow + wk * 7. * 24. * 60. * 60.
  return gps_sec + 315964800. - 18. # GPS2UNIX offset - LEAP sec

