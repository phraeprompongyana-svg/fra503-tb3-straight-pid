#!/usr/bin/env python3
"""ตัวช่วยทำแล็บ: สร้างตารางรัน (gen), รันตามตาราง (run), สรุปผล (summary)

ใช้:
  ros2 run tb3_straight_tune lab_plan gen                      # สร้าง plan.csv (ความเร็ว 0.04 0.08 0.10, 5 รอบ)
  ros2 run tb3_straight_tune lab_plan run plan.csv --bag       # รันตามตาราง ถามค่าที่ต้องวัดทีละรอบ
  ros2 run tb3_straight_tune lab_plan summary lab_results.csv  # สรุปผลเป็นตาราง

รอบที่ตั้งเงื่อนไขทางกายภาพเหมือนกันอยู่ในบล็อกเดียวกัน (ไม่ต้องเปลี่ยนตุ้มน้ำหนัก/แบตบ่อย)
ลำดับรอบในบล็อกถูกสุ่ม (กำหนด --seed ให้ซ้ำได้) ส่วนรอบเสริม (มี PID ในบล็อก 2-3) อยู่ท้ายบล็อก
"""
import argparse
import csv
import os
import random
import re
import signal
import statistics as st
import subprocess
import sys
import time
from collections import defaultdict

PLAN_COLS = ['order', 'block', 'condition', 'controller', 'v', 'rep', 'optional', 'run_id']
RESULT_COLS = ['run_id', 'block', 'condition', 'controller', 'v', 'rep', 'optional', 'kp', 'ki', 'kd',
               'batt_v', 'odom_cm', 'lat_max_cm', 'lat_end_cm', 'dyaw_deg',
               'real_lat_cm', 'real_yaw_deg', 'valid', 'note']
COND_ORDER = ['NORMAL', 'COM_OFFSET', 'BATT_TILT']
COND_TH = {'NORMAL': 'ปกติ', 'COM_OFFSET': 'CoM เยื้องข้าง', 'BATT_TILT': 'แบตเอียง'}
CTRL_TH = {'none': 'ไม่มี PID', 'pid': 'มี PID'}
SETUP_TEXT = {
    'NORMAL': 'ปกติ: แบตเตอรี่อยู่ตำแหน่งมาตรฐานของ Burger ไม่เพิ่มของ ถ่ายรูปหุ่นไว้',
    'COM_OFFSET': ('CoM เยื้องข้าง: วางตุ้มน้ำหนักที่ชั่งแล้ว (100-200 g) บนแผ่นบน เยื้องจากแนวกึ่งกลางไปด้านเดียวราว 5 cm '
                   'ตำแหน่งหน้า-หลังใกล้แกนล้อ ยึดด้วยเทป จดมวล ระยะเยื้อง และด้าน (ซ้าย/ขวา) ไว้'),
    'BATT_TILT': ('แบตเอียง: วางแบตเตอรี่เอียงด้วยลิ่มที่วัดมุมไว้ (เช่น 10 องศา) ตำแหน่งเดียวทุกรอบ ยึดไม่ให้เลื่อน '
                  'อย่ากดหรืองอตัวแบต และอย่าให้สายตึง จดมุมและด้านที่ยก'),
}
SETUP_SECONDS = 45          # เวลาวางหุ่นใหม่ + วัดผลต่อรอบ (ค่าประมาณสำหรับตารางเวลา)
TUNE_RUNS = 15
TUNE_V = 0.08


def share_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('tb3_straight_tune')
    except Exception:                                   # ไม่ได้ติดตั้ง: ใช้ config ข้างซอร์ส
        return os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')


def read_csv(path):
    with open(path, newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))


def ask(prompt):
    try:
        return input(prompt).strip()
    except (EOFError, KeyboardInterrupt):
        print()
        return 'q'


def to_float(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


# ---------------------------------------------------------------- gen
def cmd_gen(args):
    rng = random.Random(args.seed)
    spec = [('NORMAL', [('none', args.reps, False), ('pid', args.reps, False)]),
            ('COM_OFFSET', [('none', args.reps, False), ('pid', args.reps_pid, True)]),
            ('BATT_TILT', [('none', args.reps, False), ('pid', args.reps_pid, True)])]
    rows, order = [], 0
    for block, (cond, parts) in enumerate(spec, 1):
        core, opt = [], []
        for ctrl, reps, optional in parts:
            for v in args.speeds:
                for rep in range(1, reps + 1):
                    (opt if optional else core).append((cond, ctrl, v, rep, optional))
        rng.shuffle(core)
        rng.shuffle(opt)
        for cond_, ctrl, v, rep, optional in core + opt:
            order += 1
            rows.append(dict(order=order, block=block, condition=cond_, controller=ctrl, v=f'{v:.2f}', rep=rep,
                             optional=int(optional), run_id=f'{cond_}_{ctrl}_v{v:.2f}_r{rep}'))
    with open(args.out, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=PLAN_COLS)
        w.writeheader()
        w.writerows(rows)

    def secs(r):
        return args.dist / float(r['v']) + SETUP_SECONDS
    core_rows = [r for r in rows if not r['optional']]
    opt_rows = [r for r in rows if r['optional']]
    tune = TUNE_RUNS * (args.dist / TUNE_V + SETUP_SECONDS)
    print(f'เขียน {args.out}: {len(rows)} รอบ (หลัก {len(core_rows)}, เสริม {len(opt_rows)}) seed={args.seed}')
    print('บล็อก | เงื่อนไข | ตัวควบคุม | ความเร็ว | หลัก | เสริม')
    cnt = defaultdict(lambda: [0, 0])
    for r in rows:
        cnt[(r['block'], r['condition'], r['controller'], r['v'])][r['optional']] += 1
    for (b, c, k, v), (nc, no) in sorted(cnt.items(), key=lambda x: (int(x[0][0]), x[0][2], x[0][3])):
        print(f'{b:>5} | {COND_TH[c]} | {CTRL_TH[k]} | {v} | {nc} | {no}')
    print(f'เวลาโดยประมาณ (ระยะ {args.dist} m, ตั้ง/วัดผล {SETUP_SECONDS} s ต่อรอบ): '
          f'หลัก {sum(map(secs, core_rows))/60:.0f} นาที, เสริม {sum(map(secs, opt_rows))/60:.0f} นาที, '
          f'จูน PID (~{TUNE_RUNS} รอบ step test) {tune/60:.0f} นาที')


# ---------------------------------------------------------------- run
def pid_gains(path):
    vals = {}
    with open(path, encoding='utf-8') as f:
        for line in f:
            m = re.match(r'^\s*(kp|ki|kd)\s*:\s*([-+0-9.eE]+)', line.split('#', 1)[0])
            if m:
                vals[m.group(1)] = float(m.group(2))
    return vals


def read_battery():
    try:
        out = subprocess.run(['ros2', 'topic', 'echo', '--once', '--field', 'voltage', '/battery_state'],
                             capture_output=True, text=True, timeout=8).stdout
        return to_float(out.strip().splitlines()[0]) if out.strip() else None
    except Exception:
        return None


def start_bag(path):
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    if os.path.exists(path):
        path = f'{path}_{time.strftime("%H%M%S")}'
    p = subprocess.Popen(['ros2', 'bag', 'record', '-s', 'mcap', '-o', path, '/odom', '/cmd_vel'],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)                                     # รอให้ bag เริ่มจริงก่อนสั่งวิ่ง
    return p


def stop_bag(p):
    time.sleep(1.0)
    p.send_signal(signal.SIGINT)                        # ครั้งเดียว ให้ไฟล์ mcap ปิดสมบูรณ์
    try:
        p.wait(timeout=20)
    except subprocess.TimeoutExpired:
        print('  bag ยังไม่ปิด รออีกครู่ (อย่าปิดหน้าต่าง)')
        try:
            p.wait(timeout=20)
        except subprocess.TimeoutExpired:
            p.terminate()


def node_row(csv_path, run_id):
    base, ext = os.path.splitext(csv_path)
    for path in (csv_path, f'{base}_v2{ext}'):
        if os.path.exists(path):
            rows = [r for r in read_csv(path) if r.get('tag') == run_id]
            if rows:
                return rows[-1]
    return None


def cmd_run(args):
    plan = read_csv(args.plan)
    os.makedirs(args.out_dir, exist_ok=True)
    results_path = args.results or os.path.join(args.out_dir, 'lab_results.csv')
    node_csv = os.path.join(args.out_dir, 'node_log.csv')
    params_file = args.params_file or os.path.join(share_dir(), 'config', 'params.yaml')
    pid_file = args.pid_file or os.path.join(share_dir(), 'config', 'params_pid.yaml')
    done = {r['run_id'] for r in read_csv(results_path)} if os.path.exists(results_path) else set()
    todo = [r for r in plan if r['run_id'] not in done and (args.with_optional or r['optional'] != '1')]
    if any(r['controller'] == 'pid' for r in todo):
        g = pid_gains(pid_file)
        if not (g.get('kp', 0) > 0 or g.get('ki', 0) > 0 or g.get('kd', 0) > 0):
            sys.exit(f'ยังไม่ได้จูน PID: ใส่ค่า kp/ki/kd ใน {pid_file} ก่อน (ผลจาก step test) แล้วรันใหม่ '
                     f'(รอบที่ทำไปแล้วไม่ถูกทำซ้ำ)')
        print(f'ค่า PID จาก {pid_file}: {g}')
    else:
        g = {}
    print(f'เหลือ {len(todo)} รอบ (ข้ามที่ทำแล้ว {len(done)}) ผลเขียนที่ {results_path}')
    new_file = not os.path.exists(results_path)
    cur_cond = None
    for k, row in enumerate(todo, 1):
        if row['condition'] != cur_cond:
            print('\n=== ตั้งเงื่อนไขหุ่น ===')
            print(SETUP_TEXT[row['condition']])
            if ask('ตั้งเสร็จแล้วกด Enter (q = ออก): ').lower() == 'q':
                return
            cur_cond = row['condition']
        rid, v, ctrl = row['run_id'], float(row['v']), row['controller']
        print(f'\n[{k}/{len(todo)}] {rid}  ({COND_TH[row["condition"]]}, {CTRL_TH[ctrl]}, v={v} m/s, รอบ {row["rep"]})')
        ans = ask('วางหุ่นที่จุดเริ่มต้นแล้วกด Enter (s = ข้ามรอบนี้, q = ออก): ').lower()
        if ans == 'q':
            return
        if ans == 's':
            continue
        batt = read_battery()
        if batt is None:
            b = ask('อ่านแรงดันแบตเตอรี่อัตโนมัติไม่ได้ พิมพ์ค่า (V) หรือ Enter ข้าม: ')
            batt = to_float(b)
        bag = start_bag(os.path.join(args.out_dir, 'bags', rid)) if args.bag else None
        cmd = [sys.executable, '-m', 'tb3_straight_tune.run_straight_tuned', '--ros-args',
               '--params-file', params_file]
        if ctrl == 'pid':
            cmd += ['--params-file', pid_file]
        cmd += ['-p', f'v:={v}', '-p', f'dist:={args.dist}', '-p', 'test:=straight', '-p', 'tau:=0.0',
                '-p', 'accel:=0.0', '-p', 'ask_real:=false', '-p', f'csv:={node_csv}', '-p', f'tag:={rid}',
                '-p', f'trace:={os.path.join(args.out_dir, "traces", rid + ".csv")}']
        if ctrl == 'none':
            cmd += ['-p', 'kp:=0.0', '-p', 'ki:=0.0', '-p', 'kd:=0.0']
        try:
            rc = subprocess.run(cmd).returncode
        except KeyboardInterrupt:
            rc = 130
        if bag:
            stop_bag(bag)
        if rc != 0:
            print(f'รอบนี้ไม่สมบูรณ์ (รหัสออก {rc}) ไม่บันทึกลงผล')
            if ask('ลองใหม่รอบถัดไปต่อไหม Enter = ต่อ, q = ออก: ').lower() == 'q':
                return
            continue
        nr = node_row(node_csv, rid) or {}
        lat = ask('เบี่ยงข้างที่วัดด้วยตลับเมตร (cm, + = ซ้ายของเส้น, Enter = ข้าม): ')
        yaw = ask('ทิศที่หมุนที่วัดได้ (deg, + = ทวนเข็ม, Enter = ข้าม): ')
        ok = ask('ใช้รอบนี้ไหม (Enter = ใช้, n = ทิ้ง เช่น ล้อลื่น/ชนสิ่งของ): ').lower()
        note = ask('หมายเหตุ (Enter = ไม่มี): ')
        out = dict(run_id=rid, block=row['block'], condition=row['condition'], controller=ctrl, v=row['v'],
                   rep=row['rep'], optional=row['optional'],
                   kp=g.get('kp', 0.0) if ctrl == 'pid' else 0.0, ki=g.get('ki', 0.0) if ctrl == 'pid' else 0.0,
                   kd=g.get('kd', 0.0) if ctrl == 'pid' else 0.0,
                   batt_v='' if batt is None else f'{batt:.2f}',
                   odom_cm=nr.get('odom_cm', ''), lat_max_cm=nr.get('lat_max_cm', ''),
                   lat_end_cm=nr.get('lat_end_cm', ''), dyaw_deg=nr.get('dyaw_deg', ''),
                   real_lat_cm='' if to_float(lat) is None else lat, real_yaw_deg='' if to_float(yaw) is None else yaw,
                   valid='N' if ok == 'n' else 'Y', note=note)
        with open(results_path, 'a', newline='', encoding='utf-8') as f:
            w = csv.DictWriter(f, fieldnames=RESULT_COLS)
            if new_file:
                w.writeheader()
                new_file = False
            w.writerow(out)
    print('\nครบตามแผนแล้ว สรุปผลด้วย: ros2 run tb3_straight_tune lab_plan summary ' + results_path)


# ---------------------------------------------------------------- summary
def mean_sd(xs):
    xs = [x for x in xs if x is not None]
    if not xs:
        return None, None
    return st.mean(xs), (st.stdev(xs) if len(xs) > 1 else 0.0)


def cmd_summary(args):
    rows = [r for r in read_csv(args.results) if r.get('valid', 'Y').upper() != 'N']
    groups = defaultdict(list)
    for r in rows:
        groups[(r['condition'], r['controller'], float(r['v']))].append(r)

    def key(k):
        return (COND_ORDER.index(k[0]) if k[0] in COND_ORDER else 9, k[1] != 'none', k[2])
    out_rows = []
    print('เงื่อนไข | ตัวควบคุม | v | n | เบี่ยงข้าง cm (เฉลี่ย ± SD) | |เบี่ยง| เฉลี่ย cm | ทิศ deg (เฉลี่ย ± SD) | ฐาน')
    stats = {}
    for k in sorted(groups, key=key):
        g = groups[k]
        lat, yaw, basis = [], [], set()
        for r in g:
            real = to_float(r['real_lat_cm'])
            lat.append(real if real is not None else to_float(r['lat_end_cm']))
            basis.add('ตลับเมตร' if real is not None else 'odom')
            ry = to_float(r['real_yaw_deg'])
            yaw.append(ry if ry is not None else to_float(r['dyaw_deg']))
        lm, ls = mean_sd(lat)
        ym, ys = mean_sd(yaw)
        am, _ = mean_sd([abs(x) if x is not None else None for x in lat])
        stats[k] = am
        f = lambda m, s: 'n/a' if m is None else f'{m:+.2f} ± {s:.2f}'
        print(f'{COND_TH.get(k[0], k[0])} | {CTRL_TH[k[1]]} | {k[2]:.2f} | {len(g)} | {f(lm, ls)} | '
              f'{"n/a" if am is None else f"{am:.2f}"} | {f(ym, ys)} | {"+".join(sorted(basis))}')
        out_rows.append(dict(condition=k[0], controller=k[1], v=f'{k[2]:.2f}', n=len(g),
                             lat_mean=lm, lat_sd=ls, lat_abs_mean=am, yaw_mean=ym, yaw_sd=ys,
                             basis='+'.join(sorted(basis))))
    print('\nเทียบ |เบี่ยงข้าง| เฉลี่ย: ไม่มี PID -> มี PID (ดูจำนวนรอบ n ประกอบ ค่าแตกต่างน้อยกว่า SD ไม่ควรสรุปว่าต่างกัน)')
    for (c, k, v), am in sorted(stats.items(), key=lambda x: key(x[0])):
        if k != 'none' or (c, 'pid', v) not in stats:
            continue
        b = stats[(c, 'pid', v)]
        if am is not None and b is not None:
            print(f'  {COND_TH.get(c, c)} v={v:.2f}: {am:.2f} -> {b:.2f} cm')
    if args.out:
        with open(args.out, 'w', newline='', encoding='utf-8') as f:
            w = csv.DictWriter(f, fieldnames=list(out_rows[0].keys()) if out_rows else ['condition'])
            w.writeheader()
            w.writerows(out_rows)
        print(f'เขียน {args.out}')


def main(argv=None):
    ap = argparse.ArgumentParser(description='ตัวช่วยทำแล็บ: gen / run / summary')
    sub = ap.add_subparsers(dest='cmd', required=True)
    g = sub.add_parser('gen', help='สร้างตารางรัน')
    g.add_argument('--out', default='plan.csv')
    g.add_argument('--speeds', type=float, nargs='+', default=[0.04, 0.08, 0.10])
    g.add_argument('--reps', type=int, default=5, help='รอบต่อความเร็ว (รอบหลัก)')
    g.add_argument('--reps-pid', dest='reps_pid', type=int, default=3, help='รอบต่อความเร็วของรอบเสริม (มี PID ในบล็อก 2-3)')
    g.add_argument('--dist', type=float, default=1.2, help='ระยะ m (ใช้ประมาณเวลา)')
    g.add_argument('--seed', type=int, default=1)
    r = sub.add_parser('run', help='รันตามตาราง')
    r.add_argument('plan')
    r.add_argument('--out-dir', dest='out_dir', default='lab_data')
    r.add_argument('--results', default=None)
    r.add_argument('--dist', type=float, default=1.2)
    r.add_argument('--params-file', dest='params_file', default=None)
    r.add_argument('--pid-file', dest='pid_file', default=None, help='ไฟล์ kp/ki/kd ที่จูนแล้ว (ค่าเริ่มต้น config/params_pid.yaml)')
    r.add_argument('--with-optional', dest='with_optional', action='store_true', help='รวมรอบเสริม')
    r.add_argument('--bag', action='store_true', help='บันทึก rosbag (/odom, /cmd_vel) ทุกรอบ')
    s = sub.add_parser('summary', help='สรุปผล')
    s.add_argument('results')
    s.add_argument('--out', default=None, help='เขียนตารางสรุปเป็น CSV')
    args = ap.parse_args(argv)
    {'gen': cmd_gen, 'run': cmd_run, 'summary': cmd_summary}[args.cmd](args)


if __name__ == '__main__':
    main()
