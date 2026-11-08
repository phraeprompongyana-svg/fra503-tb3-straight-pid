# scripts

Helper scripts used to test and tune the system. Run them **inside the ROS container** (`root@...`) after
`source /opt/ros/jazzy/setup.bash` and `source /root/tb3_tune_ws/install/setup.bash`.

| Script | Use |
|---|---|
| `tune_step.sh` | interactive step-test loop (`source scripts/tune_step.sh; tune`) |
| `tune_straight.sh` | straight-run tuning with the tilted battery: 5 gain sets x 3 reps |
| `run_lab.sh` | generate the lab plan and run it (`bash scripts/run_lab.sh none` or `pid`) |

Stop a run with a single Ctrl+C (the node stops the robot first). Do not use Ctrl+Z: it left stale odom data in two tuning runs.
`tune_step.sh` and `tune_straight.sh` are the loops run on the real robot; `run_lab.sh` consolidates the commands used earlier and has not been run as a single script.
