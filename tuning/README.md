# tuning

Heading-PID tuning at v = 0.08 m/s, 180 cm (1.8 m).

## Step test (+10 deg at 1 s) - `step_test_summary.csv`
Transcribed from the terminal tables (8 valid rows); the raw `step_log.csv` was not saved to this repo.
Kp = 2.0 chosen: rise < 1 s, overshoot 3.8-7.5 %, sse <= 0.4 deg. Kp = 4.0 overshoots 10-15 %.
A 10 deg step saturates the 0.3 rad/s limit for Kp >= ~1.7, so tuning moved to straight runs.

## Straight-run check (battery tilt, odom lateral at the end)
Not saved as a CSV. Values read from the terminal (rep 1-2):

| gains (kp, ki, kd) | lateral (cm) |
|---|---|
| 0, 0, 0 (no PID) | -4.3, -4.5 |
| 1.0, 0, 0 | -1.0, -0.3 |
| 2.0, 0, 0 | -0.1 |
| 2.0, 0, 0.2 | 0.2 |
| 2.0, 0.1, 0 | 0.1 |

Ki and Kd could not be told apart from Kp = 2.0 alone with 1-2 runs. Locked: Kp = 2.0, Ki = 0, Kd = 0.
Two bad rows (odom jump, all-zero odom) were removed. Re-run with `scripts/tune_straight.sh` to collect more data.
