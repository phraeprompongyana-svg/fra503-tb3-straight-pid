#!/usr/bin/env python3
"""วิ่งตรง 1 รอบ: ตัวควบคุมทิศ PID + หยุดก่อนถึงเป้า (tau) + ออกตัวนุ่ม (accel) + โหมด step test

พารามิเตอร์ทั้งหมดเป็น ROS 2 parameter ของโหนด `run_straight_tuned`
ใช้: ros2 run tb3_straight_tune run_straight_tuned --ros-args --params-file <params.yaml> -p v:=0.08
ลำดับความสำคัญ: ตัวที่เขียนทีหลังในคำสั่งทับตัวก่อนหน้า (ให้ใส่ -p หลัง --params-file)

ตัวควบคุมทิศ (ทำงานเมื่อ kp, ki หรือ kd ตัวใดตัวหนึ่ง > 0; ทั้งสามเป็น 0 = ไม่มี PID, ส่ง angular.z = 0):
    e   = yaw_ref - yaw                 (rad, ห่อให้อยู่ใน -pi..pi, yaw จาก /odom)
    w   = kp*e + ki*integral(e) - kd*yaw_rate
ที่ yaw_rate คืออัตราการหมุนใน /odom (derivative on measurement จึงไม่กระชากเมื่อ ref เปลี่ยน)
ผลลัพธ์ถูกจำกัดที่ +-wmax, integral ถูกจำกัดให้ ki*integral อยู่ใน +-imax และหยุดสะสมเมื่อผลลัพธ์อิ่มตัว (anti-windup)

โหมด step test (test:=step): เริ่มวิ่งตรงที่ yaw_ref = yaw เริ่มต้น แล้วที่เวลา step_at วินาที
ขยับ yaw_ref ไป step_deg องศา บันทึกการตอบสนองของทิศ และคำนวณ overshoot / rise / settling / steady-state error
"""
import csv
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterDescriptor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistStamped

# ชื่อพารามิเตอร์ -> ค่าเริ่มต้น (ชนิดของค่าเริ่มต้นใช้แปลงค่าที่รับมา)
SPEC = dict(v=0.0, dist=1.2, mode='stamped', tau=0.0, kp=0.0, ki=0.0, kd=0.0, imax=0.1,
            accel=0.0, vmin=0.02, wmax=0.3, test='straight', step_deg=0.0, step_at=1.0,
            csv='', trace='', tag='', ask_real=False)

CSV_HEADER = ['v', 'dist_cm', 'tau', 'kp', 'accel', 'odom_cm', 'real_cm', 'lat_max_cm', 'lat_end_cm',
              'dyaw_deg', 'ki', 'kd', 'tag', 'ov_pct', 'rise_s', 'settle_s', 'sse_deg']


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class HeadingPID:
    """PID ควบคุมทิศ (ไม่พึ่ง ROS จึงทดสอบแยกได้)"""

    def __init__(self, kp, ki, kd, wmax, imax):
        self.kp, self.ki, self.kd, self.wmax, self.imax = kp, ki, kd, wmax, imax
        self.integ = 0.0                      # integral ของ e (rad*s)

    @property
    def active(self):
        return self.kp > 0 or self.ki > 0 or self.kd > 0

    def update(self, e, rate, dt):
        """e = yaw_ref - yaw (rad), rate = อัตราหมุนที่วัดได้ (rad/s), dt (s) -> angular.z (rad/s)"""
        i_term = self.ki * self.integ
        u = self.kp * e + i_term - self.kd * rate
        w = clamp(u, -self.wmax, self.wmax)
        saturated = abs(u) > self.wmax and u * e > 0      # อิ่มตัวและ e ยังดันไปทางเดิม
        if not saturated:
            self.integ += e * dt
            if self.ki > 0:
                lim = self.imax / self.ki
                self.integ = clamp(self.integ, -lim, lim)
        return w


def step_metrics(ts, yaw_deg, step_deg, step_at, band=0.05):
    """คำนวณคุณภาพการตอบสนองต่อ step ของทิศ ts (s), yaw_deg (ทิศเทียบทิศเริ่มต้น, deg)
    คืน dict: ov_pct, rise_s (10%->90%), settle_s (เข้าแถบ +-band แล้วอยู่ถาวร นับจาก step_at), sse_deg
    ค่าที่คำนวณไม่ได้ (เช่น ยังไม่เข้าที่ก่อนจบรอบ) เป็น nan"""
    nan = float('nan')
    out = dict(ov_pct=nan, rise_s=nan, settle_s=nan, sse_deg=nan)
    if step_deg == 0:
        return out
    s = 1.0 if step_deg > 0 else -1.0
    target = abs(step_deg)
    pts = [(t, s * y) for t, y in zip(ts, yaw_deg) if t >= step_at]
    if len(pts) < 3:
        return out
    t_arr = [p[0] for p in pts]
    y_arr = [p[1] for p in pts]
    out['ov_pct'] = max(0.0, (max(y_arr) - target) / target * 100.0)
    t10 = next((t for t, y in pts if y >= 0.1 * target), None)
    t90 = next((t for t, y in pts if y >= 0.9 * target), None)
    if t10 is not None and t90 is not None:
        out['rise_s'] = t90 - t10
    outside = [k for k, y in enumerate(y_arr) if abs(y - target) > band * target]
    if not outside:
        out['settle_s'] = 0.0
    elif outside[-1] < len(y_arr) - 1:
        out['settle_s'] = t_arr[outside[-1] + 1] - step_at
    t_end = t_arr[-1]
    tail = [y for t, y in pts if t >= t_end - 1.0]
    out['sse_deg'] = target - sum(tail) / len(tail)
    return out


def read_params(node):
    """declare พารามิเตอร์ทั้งหมดแล้วคืน dict ของค่าที่ใช้จริง
    dynamic_typing ทำให้เขียน `tau: 0` (จำนวนเต็ม) ในไฟล์ได้ ไม่ต้องเป็น 0.0"""
    cfg = {}
    for k, d in SPEC.items():
        node.declare_parameter(k, d, ParameterDescriptor(dynamic_typing=True))
        val = node.get_parameter(k).value
        if isinstance(d, bool):
            cfg[k] = bool(val)
        else:
            cfg[k] = type(d)(val)
    if cfg['v'] <= 0:
        raise ValueError('ต้องระบุความเร็ว v > 0 (ใส่ -p v:=0.08 หรือ v: ใน params.yaml)')
    if cfg['mode'] not in ('stamped', 'twist'):
        raise ValueError(f"mode ต้องเป็น stamped หรือ twist ไม่ใช่ {cfg['mode']!r}")
    if cfg['test'] not in ('straight', 'step'):
        raise ValueError(f"test ต้องเป็น straight หรือ step ไม่ใช่ {cfg['test']!r}")
    if min(cfg['kp'], cfg['ki'], cfg['kd']) < 0:
        raise ValueError('kp, ki, kd ต้องไม่ติดลบ')
    if cfg['test'] == 'step':
        if cfg['step_deg'] == 0:
            raise ValueError('test:=step ต้องกำหนด step_deg ไม่ใช่ 0 (เช่น -p step_deg:=10.0)')
        if not (cfg['kp'] > 0 or cfg['ki'] > 0 or cfg['kd'] > 0):
            raise ValueError('test:=step ต้องมีตัวควบคุมทิศ (kp, ki หรือ kd > 0)')
    return cfg


class Run(Node):
    def __init__(self):
        super().__init__('run_straight_tuned')
        self.a = a = read_params(self)
        self.stamped = a['mode'] == 'stamped'
        self.pid = HeadingPID(a['kp'], a['ki'], a['kd'], a['wmax'], a['imax'])
        self.pub = self.create_publisher(TwistStamped if self.stamped else Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.cb, 10)
        self.p0 = None
        self.yaw0 = 0.0
        self.yaw = 0.0
        self.wz = 0.0
        self.d = 0.0
        self.lat = 0.0
        self.max_lat = 0.0
        self.vmax = 0.0
        self.t_start = None
        self.t_prev = None
        self.t0 = time.time()
        self.timeout = a['dist'] / max(a['v'], 1e-3) * 2.0 + 10.0
        self.done = False
        self.trace = []                      # (t, v_cmd, dist, yaw_deg, ref_deg, w_cmd)
        self.create_timer(0.05, self.tick)

    def cb(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.wz = m.twist.twist.angular.z
        if self.p0 is None:
            self.p0 = (p.x, p.y)
            self.yaw0 = self.yaw
        dx, dy = p.x - self.p0[0], p.y - self.p0[1]
        self.d = math.hypot(dx, dy)
        self.lat = -dx * math.sin(self.yaw0) + dy * math.cos(self.yaw0)   # เบี่ยงด้านข้างจากแนวเริ่มต้น
        self.max_lat = max(self.max_lat, abs(self.lat))
        self.vmax = max(self.vmax, m.twist.twist.linear.x)

    def send(self, v, w=0.0):
        if self.stamped:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.twist.linear.x = v
            m.twist.angular.z = w
        else:
            m = Twist()
            m.linear.x = v
            m.angular.z = w
        self.pub.publish(m)

    def tick(self):
        a = self.a
        now = time.time()
        if self.p0 is None:
            if now - self.t0 > 5:
                self.get_logger().error('ไม่ได้รับ /odom')
                self.done = True
            return
        if self.t_start is None:
            self.t_start = now
            self.t_prev = now
        t = now - self.t_start
        dt = clamp(now - self.t_prev, 0.01, 0.2)
        self.t_prev = now
        remaining = a['dist'] - self.d
        v = a['v']
        if a['accel'] > 0:
            v = min(v, max(a['vmin'], a['accel'] * t))                                  # ออกตัวนุ่ม
            v = min(v, max(a['vmin'], math.sqrt(2 * a['accel'] * max(remaining, 0.0))))  # ชะลอก่อนหยุด
        # หยุดก่อนถึงเป้าเท่าระยะที่หุ่นจะไหลต่อ = ความเร็วที่สั่งอยู่ ณ ขณะนั้น x tau
        if remaining <= v * a['tau'] or t > self.timeout:
            self.done = True
            return
        ref = self.yaw0
        if a['test'] == 'step' and t >= a['step_at']:
            ref += math.radians(a['step_deg'])
        w = 0.0
        if self.pid.active:
            w = self.pid.update(wrap(ref - self.yaw), self.wz, dt)
        self.trace.append((t, v, self.d, math.degrees(wrap(self.yaw - self.yaw0)),
                           math.degrees(wrap(ref - self.yaw0)), w))
        self.send(v, w)


def write_trace(path, trace):
    d = os.path.dirname(path)
    if d:
        os.makedirs(d, exist_ok=True)
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['t_s', 'v_cmd', 'dist_m', 'yaw_deg', 'ref_deg', 'w_cmd'])
        for r in trace:
            w.writerow([f'{r[0]:.3f}', f'{r[1]:.4f}', f'{r[2]:.4f}', f'{r[3]:.3f}', f'{r[4]:.3f}', f'{r[5]:.4f}'])


def append_csv(path, row):
    """ต่อท้าย CSV ถ้าไฟล์เดิมมีหัวคอลัมน์ต่างจากเวอร์ชันนี้ จะเขียนลงไฟล์ใหม่ (ชื่อต่อท้าย _v2) เพื่อไม่ให้คอลัมน์เลื่อน"""
    if os.path.exists(path):
        with open(path, encoding='utf-8') as f:
            first = f.readline().strip()
        if first != ','.join(CSV_HEADER):
            base, ext = os.path.splitext(path)
            path = f'{base}_v2{ext}'
            print(f'คำเตือน: CSV เดิมมีหัวคอลัมน์ต่างจากเวอร์ชันนี้ เขียนลง {path} แทน', file=sys.stderr)
    new = not os.path.exists(path)
    d = os.path.dirname(path)
    if d:
        os.makedirs(d, exist_ok=True)
    with open(path, 'a', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        if new:
            w.writerow(CSV_HEADER)
        w.writerow(row)
    return path


def fmt(x, nd=2):
    return 'nan' if x != x else f'{x:.{nd}f}'


def main():
    # ปิดตัวจัดการ SIGINT ของ rclpy เพื่อให้ Ctrl+C เป็น KeyboardInterrupt ปกติ
    # (ไม่งั้น context ถูกปิดก่อน และส่งคำสั่งหยุดตอนถูกขัดจังหวะไม่ได้)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        n = Run()
    except ValueError as e:
        print(f'ผิดพลาด: {e}', file=sys.stderr)
        rclpy.shutdown()
        sys.exit(2)
    a = n.a
    print('พารามิเตอร์ที่ใช้ (ตรวจว่าตรงกับที่ตั้งใจ ชื่อที่พิมพ์ผิดใน params.yaml จะถูกข้ามเงียบๆ):')
    for k in SPEC:
        print(f'  {k:9s} = {a[k]!r}')
    print('  ตัวควบคุมทิศ:', 'PID' if n.pid.active else 'ไม่มี (open-loop, angular.z = 0)')

    interrupted = False
    try:
        while rclpy.ok() and not n.done:
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        interrupted = True
        print('\nถูกขัดจังหวะ (Ctrl+C): ส่งคำสั่งหยุดแล้วออก ไม่บันทึกผลรอบนี้', file=sys.stderr)
    for _ in range(20):
        n.send(0.0, 0.0)
        time.sleep(0.05)
    if interrupted:
        n.destroy_node()
        rclpy.shutdown()
        sys.exit(130)
    end = time.time() + 2.0                               # รอให้หุ่นนิ่งแล้วอ่านค่าสุดท้าย
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)

    final_cm = n.d * 100
    dyaw = math.degrees(wrap(n.yaw - n.yaw0)) if n.p0 else 0.0
    print(f"v={a['v']} tau={a['tau']} kp={a['kp']} ki={a['ki']} kd={a['kd']} accel={a['accel']} | "
          f"odom ระยะ={final_cm:.1f} cm (เป้า {a['dist']*100:.0f}) เบี่ยงสูงสุด={n.max_lat*100:.1f} cm "
          f"เบี่ยงตอนจบ={n.lat*100:.1f} cm หมุน={dyaw:.1f} deg vmax={n.vmax:.3f} m/s")
    sm = dict(ov_pct=float('nan'), rise_s=float('nan'), settle_s=float('nan'), sse_deg=float('nan'))
    if a['test'] == 'step':
        sm = step_metrics([r[0] for r in n.trace], [r[3] for r in n.trace], a['step_deg'], a['step_at'])
        print(f"step {a['step_deg']:+.1f} deg ที่ t={a['step_at']} s: overshoot={fmt(sm['ov_pct'], 1)} % "
              f"rise(10-90%)={fmt(sm['rise_s'])} s settling(+-5%)={fmt(sm['settle_s'])} s "
              f"steady-state error={fmt(sm['sse_deg'])} deg (nan = ยังไม่เข้าที่ก่อนจบรอบ ลองเพิ่ม dist)")
    if a['trace']:
        write_trace(a['trace'], n.trace)
        print(f"บันทึก trace: {a['trace']}")
    real = ''
    if a['ask_real']:
        try:
            real = input('ระยะที่วัดจริงด้วยตลับเมตร (cm) กด Enter ถ้าข้าม: ').strip()
        except EOFError:          # ไม่มี stdin (เช่นรันผ่าน ros2 launch) ใช้ ros2 run แทนถ้าต้องการถามระยะจริง
            print('ไม่มี stdin จึงข้ามการถามระยะจริง (ใช้ ros2 run แทน ros2 launch)', file=sys.stderr)
    if a['csv']:
        append_csv(a['csv'], [a['v'], a['dist'] * 100, a['tau'], a['kp'], a['accel'], f'{final_cm:.2f}', real,
                              f'{n.max_lat*100:.2f}', f'{n.lat*100:.2f}', f'{dyaw:.2f}', a['ki'], a['kd'],
                              a['tag'], fmt(sm['ov_pct']), fmt(sm['rise_s']), fmt(sm['settle_s']),
                              fmt(sm['sse_deg'])])
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
