#!/usr/bin/env python3
"""สรุปผล tune_log.csv: จัดกลุ่มตามชุดค่า (tau, kp, accel) เรียงจากผลดีที่สุด
ใช้ระยะที่วัดจริงถ้ามี ไม่มีก็ใช้ odom

ใช้: ros2 run tb3_straight_tune summarize_tune tune_log.csv
     ros2 run tb3_straight_tune summarize_tune tune_log.csv --write-params [โฟลเดอร์]
       (เขียนชุดค่าที่ดีที่สุดของแต่ละความเร็วเป็นไฟล์ params_best_v0.06.yaml ฯลฯ
        ในรูปแบบ ROS 2 parameter file ใช้กับ --params-file ได้ทันที)
"""
import argparse
import csv
import os
import statistics as st
from collections import defaultdict


def main(argv=None):
    ap = argparse.ArgumentParser(description='สรุปผล tune_log.csv')
    ap.add_argument('csv_file', nargs='?', default='tune_log.csv')
    ap.add_argument('--write-params', nargs='?', const='.', default=None, metavar='DIR',
                    help='เขียนชุดค่าที่ดีที่สุดต่อความเร็วเป็นไฟล์ params_best_v<ความเร็ว>.yaml ในโฟลเดอร์ DIR (ค่าเริ่มต้น .)')
    args = ap.parse_args(argv)

    g = defaultdict(list)
    with open(args.csv_file, encoding='utf-8') as f:
        for r in csv.DictReader(f):
            dist = float(r['dist_cm'])
            use_real = r['real_cm'].strip() != ''
            val = float(r['real_cm']) if use_real else float(r['odom_cm'])
            g[(float(r['v']), float(r['tau']), float(r['kp']), float(r['accel']))].append(
                (val - dist, float(r['lat_max_cm']), abs(float(r['dyaw_deg'])), use_real))
    rows = []
    for k, v in g.items():
        err = [x[0] for x in v]
        rows.append((abs(st.mean(err)), k, st.mean(err), st.pstdev(err) if len(err) > 1 else 0.0,
                     st.mean(x[1] for x in v), st.mean(x[2] for x in v), len(v), all(x[3] for x in v)))
    rows.sort(key=lambda r: (r[0], r[3]))
    print('v    tau   kp   accel | error เฉลี่ย  SD    เบี่ยงข้างเฉลี่ย  หมุนเฉลี่ย  n  ฐาน')
    for _, k, m, s, lat, yaw, n, real in rows:
        print(f'{k[0]:.2f} {k[1]:.2f}  {k[2]:.1f}  {k[3]:.2f}  | {m:+6.2f} cm  {s:4.2f}  {lat:6.2f} cm        {yaw:5.1f} deg  {n}  {"วัดจริง" if real else "odom"}')

    if args.write_params:
        best = {}
        for row in rows:                       # rows เรียงดีที่สุดก่อน เก็บตัวแรกของแต่ละความเร็ว
            best.setdefault(row[1][0], row)
        os.makedirs(args.write_params, exist_ok=True)
        for v in sorted(best):
            _, k, m, s, lat, yaw, n, real = best[v]
            path = os.path.join(args.write_params, f'params_best_v{v:.2f}.yaml')
            lines = [f'# สร้างโดย summarize_tune จากผลใน {args.csv_file}',
                     f'# error {m:+.2f} cm, SD {s:.2f}, n={n}, ฐาน: {"วัดจริง" if real else "odom"} (ตรวจก่อนใช้)',
                     'run_straight_tuned:',
                     '  ros__parameters:',
                     f'    v: {v}', f'    tau: {k[1]}', f'    kp: {k[2]}', f'    accel: {k[3]}']
            with open(path, 'w', encoding='utf-8') as f:
                f.write('\n'.join(lines) + '\n')
            print(f'เขียน {path}')
        print('ใช้งาน: ros2 run tb3_straight_tune run_straight_tuned --ros-args '
              '--params-file <params.yaml> --params-file params_best_v0.06.yaml')


if __name__ == '__main__':
    main()
