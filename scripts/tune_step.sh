#!/usr/bin/env bash
# Interactive step-test tuning loop (run INSIDE the ROS container after sourcing ROS + the workspace).
# Usage:  source scripts/tune_step.sh ; tune
# Each round: enter "kp ki kd" (Enter = keep, s = change step angle, q = quit), place the robot, press Enter.
# Results append to tune_log.csv, traces to tune_trace_<round>.csv.
KP=2.0; KI=0.0; KD=0.0; STEP=5.0; V=0.08; DIST=1.8; N=1
P=$(ros2 pkg prefix tb3_straight_tune)/share/tb3_straight_tune/config/params.yaml
tune() {
  while true; do
    echo "--- round $N | kp=$KP ki=$KI kd=$KD step=${STEP} deg v=$V ---"
    read -r -p "new 'kp ki kd' (Enter = keep, s = step angle, q = quit): " a
    case "$a" in
      q) break ;;
      s) read -r -p "step angle (deg, negative = turn right): " STEP; continue ;;
      "") ;;
      *) set -- $a
         if [ $# -ne 3 ]; then echo "need 3 values, e.g. 2 0 0.1"; continue; fi
         KP=$1; KI=$2; KD=$3 ;;
    esac
    read -r -p "place the robot at the start, press Enter to run (Ctrl+C = stop): " _
    ros2 run tb3_straight_tune run_straight_tuned --ros-args --params-file "$P" \
      -p v:=$V -p dist:=$DIST -p test:=step -p step_deg:=$STEP \
      -p kp:=$KP -p ki:=$KI -p kd:=$KD \
      -p tag:=t${N}_kp${KP}_ki${KI}_kd${KD} -p csv:=tune_log.csv -p trace:=tune_trace_${N}.csv
    N=$((N+1))
    [ -f tune_log.csv ] && (head -1 tune_log.csv; tail -n 5 tune_log.csv) | column -s, -t
  done
}
