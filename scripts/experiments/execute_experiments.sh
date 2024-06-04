#!/bin/bash

. ./experiments/utils.sh

#run_experiment "visualize_networks"
#run_experiment "visualize_queries"

#run_experiment "alt_landmarks_number"



run_experiment "alt_dist_landmarks_vs_threshold"
run_experiment "alt_hops_landmarks_vs_threshold"

run_experiment "alt_avoid"
run_experiment "alt_farthest"
run_experiment "alt_dist_threshold_query_vs_time"
run_experiment "alt_hops_threshold_query_vs_time"
run_experiment "adaptive_dist_threshold_query_vs_time"
run_experiment "adaptive_hops_threshold_query_vs_time"

run_experiment "alt_compare_methods"




