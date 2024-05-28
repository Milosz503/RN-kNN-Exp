#!/bin/bash

. ./experiments/utils.sh
OUTPUT=$output_path/results/alt_landmarks_number
mkdir -p "$OUTPUT"
run_command "${exe_path}/nd_knn -c distance -g $idx_dir/ME.bin -q 2048 -k 1 -d 500000000 -l 16 -v 0 -r 3 -x 6 -f $OUTPUT"
python ./experiments/visualization/alt_landmarks_number.py "$OUTPUT/results_output.csv" "$OUTPUT"
