#!/usr/bin/env bash
# Generate and run the Assignment 9 plan (run INSIDE the ROS container, in the folder where lab_data/ should go).
# Lock the tuned gains first in tb3_straight_tune/config/params_pid.yaml (Kp=2.0, Ki=0, Kd=0).
set -e
ros2 run tb3_straight_tune lab_plan gen --dist 1.8 --out plan.csv
awk -F, 'NR==1 || $4=="none"' plan.csv > plan_none.csv   # no-PID runs (45)
awk -F, 'NR==1 || $4=="pid"'  plan.csv > plan_pid.csv    # with-PID runs (33)
case "${1:-pid}" in
  none) ros2 run tb3_straight_tune lab_plan run plan_none.csv --dist 1.8 --bag ;;
  pid)  ros2 run tb3_straight_tune lab_plan run plan_pid.csv  --dist 1.8 --bag --with-optional ;;
esac
ros2 run tb3_straight_tune lab_plan summary lab_data/lab_results.csv --out summary.csv
