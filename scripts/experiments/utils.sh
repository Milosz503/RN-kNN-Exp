#!/bin/bash

. globalVariables
# Function to run the command in the background and log its output
run_command() {
  local COMMAND=$1

  LOG_FILE=$output_path/log.txt
  # Run the command in the background and get its PID
  $COMMAND > $LOG_FILE 2>&1 &
  local PID=$!

  echo "PID: $PID"
  echo "Command '$COMMAND' is running in the background."
  echo "Output is being saved to $output_path/log.txt."
  echo ""
  echo "------"
  echo ""

  tail -f $LOG_FILE &
  TAIL_PID=$!
  wait $PID
  kill $TAIL_PID

}

run_experiment() {
  local EXPERIMENT=$1

  run_command "python ./experiments/visualization/$EXPERIMENT.py "$output_path" "$idx_dir" "$exe_path""

}

