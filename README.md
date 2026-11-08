# fra503-tb3-straight-pid

FRA503 (AMR) Assignment 9 — TurtleBot3 Burger (ROS 2 Jazzy): วิ่งตรง 180 cm แบบมี/ไม่มี heading PID ภายใต้เงื่อนไขปกติ, CoM เยื้องข้าง และแบตเอียง ที่ความเร็ว 0.04 / 0.08 / 0.10 m/s

## โครงสร้าง

| โฟลเดอร์ | เนื้อหา |
|---|---|
| `tb3_straight_tune/` | แพ็กเกจ ROS 2 (ament_python): `run_straight_tuned`, `lab_plan`, `tune_sweep`, `summarize_tune` (รายละเอียดใน README ของแพ็กเกจ) |
| `data/assignment9/` | ผลรอบไม่มี PID 45 รอบ (`lab_results.csv`, `node_log.csv`) |
| `analysis/an.py` | สรุปค่าเฉลี่ย ± SD ต่อเซลล์ (เงื่อนไข × ความเร็ว) |

ค่า PID ที่จูนแล้ว: **Kp = 2.0, Ki = 0, Kd = 0** (`tb3_straight_tune/config/params_pid.yaml`) รอบมี PID ของแล็บ (33 รอบ) ยังไม่ได้เก็บ

## ติดตั้งบนหุ่น (Docker `tb3dev`, ROS 2 Jazzy)

ทำตามลำดับ โดยแยกว่าคำสั่งไหนรันที่ **เครื่องโฮสต์** และไหนรัน **ในคอนเทนเนอร์** (prompt ในคอนเทนเนอร์ขึ้นต้น `root@`) ห้ามรัน `colcon` ที่โฮสต์ เพราะไม่มี ROS

### 1) โฮสต์: ดึงโค้ดและคัดลอกเข้าคอนเทนเนอร์
```bash
git clone https://github.com/<ชื่อผู้ใช้>/fra503-tb3-straight-pid.git
cd fra503-tb3-straight-pid

# ลบของเก่าก่อน ไม่งั้น docker cp จะซ้อนโฟลเดอร์ (…/tb3_straight_tune/tb3_straight_tune)
sudo docker exec tb3dev rm -rf /root/tb3_tune_ws/src/tb3_straight_tune
sudo docker exec tb3dev mkdir -p /root/tb3_tune_ws/src
sudo docker cp tb3_straight_tune tb3dev:/root/tb3_tune_ws/src/tb3_straight_tune
```
ถ้ายังไม่มีสิทธิ์ใช้ docker โดยไม่ใช้ `sudo` ให้ใส่ `sudo` นำหน้า `docker` ทุกคำสั่งตามข้างบน

### 2) เข้าคอนเทนเนอร์แล้ว build
```bash
sudo docker exec -it tb3dev bash          # โฮสต์ → เข้าคอนเทนเนอร์
```
```bash
# ในคอนเทนเนอร์ (root@...)
source /opt/ros/jazzy/setup.bash
cd /root/tb3_tune_ws
colcon build --symlink-install --packages-select tb3_straight_tune
source install/setup.bash
ros2 pkg prefix tb3_straight_tune         # ต้องพิมพ์ path ออกมา
```
ทุกเทอร์มินัลใหม่ในคอนเทนเนอร์ต้อง `source /opt/ros/jazzy/setup.bash` และ `source /root/tb3_tune_ws/install/setup.bash` อีกครั้ง

### 3) ตรวจก่อนสั่งหุ่น
```bash
export ROS_DOMAIN_ID=77                   # ให้ตรงกับหุ่น
ros2 topic hz /odom                       # ควรได้ประมาณ 20 Hz
ros2 topic info /cmd_vel                  # Jazzy ใช้ geometry_msgs/TwistStamped
```
ต้องรัน bringup ของหุ่นไว้แล้ว (หุ่นต้องส่ง `/odom` และรับ `/cmd_vel`) และไม่มีโปรแกรมอื่น (teleop, Nav2) ส่ง `/cmd_vel` พร้อมกัน

### 4) ทดลองวิ่งตรงทีละรอบ
```bash
P=$(ros2 pkg prefix tb3_straight_tune)/share/tb3_straight_tune/config
# ไม่มี PID
ros2 run tb3_straight_tune run_straight_tuned --ros-args --params-file $P/params.yaml -p v:=0.04 -p dist:=1.8
# มี PID (Kp = 2.0)
ros2 run tb3_straight_tune run_straight_tuned --ros-args --params-file $P/params.yaml --params-file $P/params_pid.yaml -p v:=0.04 -p dist:=1.8
```
เริ่มที่ความเร็วต่ำสุด (0.04 m/s) และเตรียมกด `Ctrl+C` ครั้งเดียวไว้ตลอด โหนดจะสั่งหยุดหุ่นก่อนออก (กดซ้ำหรือ Ctrl+Z ทำให้ค่า odom เพี้ยน)

### 5) รันตามแผนทั้งชุด (`lab_plan`)
```bash
cd /root
ros2 run tb3_straight_tune lab_plan gen --dist 1.8 --out plan.csv
awk -F, 'NR==1 || $4=="none"' plan.csv > plan_none.csv      # รอบไม่มี PID
awk -F, 'NR==1 || $4=="pid"'  plan.csv > plan_pid.csv       # รอบมี PID
ros2 run tb3_straight_tune lab_plan run plan_pid.csv --dist 1.8 --bag --with-optional
ros2 run tb3_straight_tune lab_plan summary lab_data/lab_results.csv --out summary.csv
```
`lab_plan run` บอกให้ตั้งเงื่อนไขหุ่นทีละบล็อก อ่านแรงดันแบต สั่งวิ่ง แล้วถามค่าที่วัดด้วยตลับเมตร ทำต่อได้ถ้าหยุดกลางคัน (ข้ามรอบที่บันทึกแล้ว) และจะไม่ยอมรันรอบมี PID ถ้าค่าใน `params_pid.yaml` เป็น 0 ทั้งหมด

### 6) สำรองผลออกมาที่โฮสต์ (รันที่โฮสต์)
```bash
sudo docker cp tb3dev:/root/lab_data ./lab_backup
```

## หมายเหตุ

- PID คุมเฉพาะทิศจาก `/odom` (encoder ล้อ) จึงแก้ไม่ได้กับสิ่งที่ odom มองไม่เห็น เช่น ล้อลื่น
- ค่าจูนได้จาก step test +10° และการวิ่งตรงในเงื่อนไขแบตเอียงที่ 0.08 m/s ข้อมูลต่อชุดค่ามี 1–2 รอบ
- ยังไม่มีไฟล์ LICENSE ใน repo (ใน `package.xml` ระบุ MIT)
