#!/bin/bash

. ./experiments/utils.sh

run_command "${exe_path}/nd_knn -c distance -g $idx_dir/ME.bin -q 1024 -k 1 -d 500000000 -l 16 -v 0 -r 1 -x 6"