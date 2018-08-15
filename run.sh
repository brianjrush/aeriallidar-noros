#!/bin/bash
set -e
OPTIND=1
j=8

usage() {
  echo "Usage: `basename "$0"` [options] dataflash_log output_dir cloud1.ply cloud2.ply..."
  echo " -d  delete clouds from RAM after processing"
  echo " -h  display this message"
  echo " -j  the number of jobs to run (BROKEN)"
  echo " -r  load all clouds into RAM"
}

load_to_ram=false
del=false
while getopts "hj:rd" OPTION; do
  case "$OPTION" in
    h) usage;;
	j) j="$OPTARG";;
	d) del=true;;
	r) load_to_ram=true;;
	\?) echo "Unknown option -$OPTARG"; exit 1;;
	:) echo "Missing option argument of -$OPTARG"; exit 1;;
  esac
done
shift $(($OPTIND-1))
if [ "$#" -lt 2 ]; then echo "Not enough arguments"; usage; exit 1; fi
dataflash_log="$1"
outdir="$2"
mkdir -p "$outdir"

shift 2

TMPDIR="/tmp/lidar"
mkdir -p "$TMPDIR"

#tfnpy="$TMPDIR/tf.npy"
#stampsnpy="$TMPDIR/stamp.npy"
json="$TMPDIR/tf.json"
if [ ${dataflash_log: -5} == '.json' ]; then
  json="$dataflash_log"
  echo "Using existing transforms"
else
  echo "Parsing transforms from $dataflash_log"
  python3 dataflash_converter.py "$dataflash_log" "$json"
  echo "Done parsing transforms"
fi

if $load_to_ram; then
  echo "Loading clouds into RAM..."
  mkdir -p "$TMPDIR/clouds"
  count=0
  for file in "$@"; do
	cp "$file" "$TMPDIR/clouds/"
	if [ $(($count%500)) -eq 0 ]; then echo "$count/$# files moved"; fi
	count=$(($count+1))
  done
  echo "$count/$# files moved"
  echo "Done"
fi

echo "Transforming clouds with 8 jobs"
if $load_to_ram; then
  python3 pointcloud_transformer_parallel.py "$outdir" "$json" $TMPDIR/clouds/*
else
  python3 pointcloud_transformer_parallel.py "$outdir" "$json" "$@"
fi
if $del; then rm -r "$TMPDIR"; fi
echo "Done"

