#!/bin/bash
set -e
OPTIND=1
j=8

usage() {
  echo "Usage: `basename "$0"` [options] dataflash_log output_dir cloud1.ply cloud2.ply..."
  echo " -d  delete clouds from RAM after processing"
  echo " -h  display this message"
  echo " -j  the number of jobs to run"
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

tfnpy="$TMPDIR/tf.npy"
stampsnpy="$TMPDIR/stamp.npy"
csv="$TMPDIR/tf.csv"

echo "Parsing transforms from $dataflash_log"
python3 dataflash_converter.py "$dataflash_log" "$csv"
python3 csv_to_npy.py "$csv" "$stampsnpy" "$tfnpy"
echo "Done parsing transforms"

if $load_to_ram; then
  echo "Loading clouds into RAM..."
  mkdir -p "$TMPDIR/clouds"
  count=1
  for file in "$@"; do
	cp "$file" "$TMPDIR/clouds/"
	count=$(($count+1))
	if [ $(($count%500)) -eq 0 ]; then echo "$count/$# files moved"; fi
  done
  echo "$count/$# files moved"
  echo "Done"
fi

echo "Transforming clouds with $j jobs"
if $load_to_ram; then
  parallel -j "$j" python3 pointcloud_transformer_parallel.py {} "$stampsnpy" "$tfnpy" "$outdir/{/}" ::: $TMPDIR/clouds/*
else
  parallel -j "$j" python3 pointcloud_transformer_parallel.py {} "$stampsnpy" "$tfnpy" "$outdir/{/}" ::: "$@"
fi
if $del; then rm -r "$TMPDIR"; fi
echo "Done"

