#!/usr/bin/env python3
"""ทดลองหลายชุดค่าอัตโนมัติ: แต่ละรอบรอให้วางหุ่นที่จุดเริ่ม แล้วรันโหนดและถามระยะที่วัดจริง

ใช้: ros2 run tb3_straight_tune tune_sweep 0.06 1.2
     REPS: --reps 3   ชุดค่า: --taus 0.0 0.08 0.12 --kps 0.0 1.0 --accels 0.0 0.05
     ดูคำสั่งที่จะรันโดยไม่สั่งหุ่น: --dry-run
ค่า v, dist, tau, kp, accel ส่งด้วย -p จึงทับค่าในไฟล์พารามิเตอร์ (ค่าอื่นเช่น mode, vmin, wmax อ่านจากไฟล์)
"""
import argparse
import os
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory


def default_params_file():
    return os.path.join(get_package_share_directory('tb3_straight_tune'), 'config', 'params.yaml')


def main(argv=None):
    ap = argparse.ArgumentParser(description='กวาดชุดค่า tau/kp/accel แล้วบันทึกผลลง CSV')
    ap.add_argument('v', type=float, help='ความเร็ว m/s')
    ap.add_argument('dist', type=float, nargs='?', default=1.2, help='ระยะ m (ค่าเริ่มต้น 1.2)')
    ap.add_argument('--reps', type=int, default=3, help='จำนวนรอบต่อชุดค่า')
    ap.add_argument('--csv', default='tune_log.csv')
    ap.add_argument('--params-file', default=None, help='ไฟล์พารามิเตอร์ (ค่าเริ่มต้น: config/params.yaml ของแพ็กเกจ)')
    ap.add_argument('--taus', type=float, nargs='+', default=[0.0, 0.08, 0.12, 0.16])
    ap.add_argument('--kps', type=float, nargs='+', default=[0.0, 1.0])
    ap.add_argument('--accels', type=float, nargs='+', default=[0.0, 0.05])
    ap.add_argument('--dry-run', action='store_true', help='พิมพ์คำสั่งแต่ไม่รัน')
    args = ap.parse_args(argv)

    params_file = args.params_file or default_params_file()
    total = len(args.taus) * len(args.kps) * len(args.accels) * args.reps
    print(f'ทั้งหมด {total} รอบ ใช้ไฟล์พารามิเตอร์ {params_file}')
    n = 0
    for tau in args.taus:
        for kp in args.kps:
            for acc in args.accels:
                for i in range(1, args.reps + 1):
                    n += 1
                    cmd = [sys.executable, '-m', 'tb3_straight_tune.run_straight_tuned',
                           '--ros-args', '--params-file', params_file,
                           '-p', f'v:={args.v}', '-p', f'dist:={args.dist}',
                           '-p', f'tau:={tau}', '-p', f'kp:={kp}', '-p', f'accel:={acc}',
                           '-p', f'csv:={args.csv}', '-p', 'ask_real:=true']
                    print(f'=== [{n}/{total}] v={args.v} tau={tau} kp={kp} accel={acc} รอบ {i}/{args.reps} ===')
                    if args.dry_run:
                        print('  ' + ' '.join(cmd))
                        continue
                    try:
                        ans = input('วางหุ่นที่จุดเริ่มต้นแล้วกด Enter (พิมพ์ q เพื่อออก): ')
                    except (EOFError, KeyboardInterrupt):
                        print()
                        return
                    if ans.strip().lower() == 'q':
                        return
                    try:
                        subprocess.run(cmd, check=False)
                    except KeyboardInterrupt:
                        print('\nหยุดการกวาดค่า')
                        return


if __name__ == '__main__':
    main()
