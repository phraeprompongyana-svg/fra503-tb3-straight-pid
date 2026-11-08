#!/usr/bin/env bash
# Straight-line tuning with the battery-tilt disturbance (run INSIDE the ROS container, v=0.08 m/s, 1.8 m).
# 5 gain sets x 3 reps, interleaved so battery drain does not favour one set.
# Results append to tune_straight.csv, traces to ts_*.csv. Stop with Ctrl+C only.
# Usage:  bash scripts/tune_straight.sh        (or source it and call: run <rep> <kp> <ki> <kd>)
P=$(ros2 pkg prefix tb3_straight_tune)/share/tb3_straight_tune/config/params.yaml
run() {  # $1 rep, $2 kp, $3 ki, $4 kd
  read -r -p "[rep $1] kp=$2 ki=$3 kd=$4 place the robot at the start, press Enter: " _
  ros2 run tb3_straight_tune run_straight_tuned --ros-args --params-file $P \
    -p v:=0.08 -p dist:=1.8 -p test:=straight -p kp:=$2 -p ki:=$3 -p kd:=$4 \
    -p tag:=ts_kp$2_ki$3_kd$4_r$1 -p csv:=tune_straight.csv -p trace:=ts_kp$2_ki$3_kd$4_r$1.csv
}
if [ "${BASH_SOURCE[0]}" = "$0" ]; then
  for rep in 1 2 3; do
    for cfg in "0 0 0" "1.0 0 0" "2.0 0 0" "2.0 0 0.2" "2.0 0.1 0"; do
      set -- $cfg; run $rep $1 $2 $3
    done
  done
  column -s, -t tune_straight.csv
fi
