# tb3_straight_tune v0.2

แพ็กเกจ ROS 2 (ament_python) สำหรับ TurtleBot3 Burger (Jazzy): วิ่งตรงระยะที่กำหนด พร้อม PID ควบคุมทิศ, โหมด step test, และตัวช่วยทำแล็บ (ตารางรัน + สรุปผล)

> สถานะ: colcon build และรันกับหุ่นจริงแล้ว (รอบไม่มี PID 45 รอบ, จูน Kp = 2.0) ให้ลองที่ความเร็วต่ำก่อนและเตรียมกดหยุด (Ctrl+C) ได้ทันที
> /odom ที่วัดได้ในเครื่องเดิมคือ 20 Hz

## ติดตั้ง
```bash
cd ~/ws/src && cp -r /path/to/tb3_straight_tune .
cd ~/ws && colcon build --symlink-install --packages-select tb3_straight_tune
source install/setup.bash
```
(ถ้าอยู่ใน Docker ให้ทำทั้งหมดในคอนเทนเนอร์ และ `source install/setup.bash` ทุกเทอร์มินัลใหม่)

## โหนดวิ่งตรง
```bash
# ไม่มี PID (kp=ki=kd=0)
ros2 run tb3_straight_tune run_straight_tuned --ros-args --params-file $(ros2 pkg prefix tb3_straight_tune)/share/tb3_straight_tune/config/params.yaml -p v:=0.08
# มี PID
ros2 run tb3_straight_tune run_straight_tuned --ros-args --params-file .../params.yaml -p v:=0.08 -p kp:=2.0
```
พารามิเตอร์ที่ใส่ `-p` หลังสุดจะทับค่าในไฟล์ ดูความหมายทั้งหมดใน `config/params.yaml`

ตัวควบคุมทิศ: `e = yaw_ref - yaw`, `w = kp*e + ki*integral(e) - kd*yaw_rate` (อนุพันธ์จากค่าวัด), จำกัด `|w| <= wmax`, ส่วน integral จำกัดที่ `imax` และหยุดสะสมเมื่ออิ่มตัว (anti-windup) `yaw` มาจาก /odom

Ctrl+C จะสั่งหยุดหุ่นก่อนออก และรอบนั้นไม่ถูกบันทึก

## Step test (ใช้จูนค่า PID)
```bash
ros2 run tb3_straight_tune run_straight_tuned --ros-args --params-file .../params.yaml \
  -p v:=0.08 -p dist:=2.0 -p test:=step -p step_deg:=10.0 -p kp:=2.0 -p trace:=step_kp2.csv -p csv:=step_log.csv -p tag:=kp2
```
ที่ `step_at` วินาที (ค่าเริ่มต้น 1.0) เป้าหมายทิศเปลี่ยนไป `step_deg` องศา โหนดคำนวณ overshoot (%), rise time 10-90%, settling time (+-5%), steady-state error (deg) ลง CSV (`ov_pct, rise_s, settle_s, sse_deg`; NaN = ยังไม่เข้าแถบ) และบันทึกเส้นทาง yaw/คำสั่งต่อเวลาใน `trace`

### ขั้นตอนจูน (ที่ v = 0.08 m/s เงื่อนไขปกติ)
1. P อย่างเดียว: เริ่ม kp = 0.5 แล้วเพิ่มทีละขั้น (0.5, 1, 2, 4, 8) ดู overshoot และการสั่น
2. เลือก kp ที่เร็วพอและ overshoot ต่ำ (ถ้าพบ kp ที่เริ่มสั่นต่อเนื่อง Ku ให้ใช้ kp ~ 0.5 Ku)
3. ถ้ายังมี sse ค้าง ค่อยเติม ki เล็กน้อย; ถ้า overshoot สูง เติม kd
4. ล็อกค่าที่ได้ใส่ `config/params_pid.yaml` แล้วใช้ชุดเดียวกับทุกความเร็วและทุกเงื่อนไข (plant ของทิศใกล้เคียง integrator จึงไม่ขึ้นกับความเร็วมาก)

ตัวอย่างค่าจากการจำลองบน stub (ไม่ใช่ข้อมูลหุ่นจริง): kp=2 ได้ overshoot ~1%, settle ~1 s

อย่าแก้ PID ภายในของมอเตอร์ Dynamixel บนหุ่นที่ใช้ร่วมกัน

## ตัวช่วยทำแล็บ `lab_plan`
```bash
ros2 run tb3_straight_tune lab_plan gen --out plan.csv                 # ความเร็ว 0.04 0.08 0.10, 5 รอบหลัก, 3 รอบเสริม
ros2 run tb3_straight_tune lab_plan run plan.csv --bag --with-optional # รันตามตาราง
ros2 run tb3_straight_tune lab_plan summary lab_data/lab_results.csv --out summary.csv
```
- `gen`: 78 รอบ = หลัก 60 (ปกติ ทั้งมี/ไม่มี PID, และ CoM/แบตเอียง ไม่มี PID) + เสริม 18 (มี PID ใน CoM/แบตเอียง) จัดเป็น 3 บล็อกตามเงื่อนไขกายภาพ สุ่มลำดับในบล็อก (`--seed`)
- `run`: บอกให้ตั้งเงื่อนไขหุ่นทีละบล็อก, อ่านแรงดันแบต, เปิด rosbag (`--bag`), สั่งวิ่ง, แล้วถามค่าที่วัดด้วยตลับเมตร (เบี่ยงข้าง cm, ทิศ deg) เขียน `lab_data/lab_results.csv` ทีละรอบ ทำต่อได้ถ้าหยุดกลางคัน (ข้ามรอบที่บันทึกแล้ว) ต้องใส่ค่า PID ใน `config/params_pid.yaml` (หรือ `--pid-file`) ก่อน ไม่งั้นจะไม่ยอมรันรอบ "มี PID"
- `summary`: ค่าเฉลี่ย +- SD ต่อเซลล์ (ไม่รวมรอบที่ valid = N) ใช้ค่าตลับเมตรก่อน ถ้าไม่มีใช้ค่า odom (แสดงในคอลัมน์ฐาน) และเทียบ |เบี่ยง| ระหว่างไม่มี PID กับมี PID

odom มองไม่เห็นล้อลื่น ให้ถือค่าตลับเมตรเป็นหลัก

## เครื่องมืออื่น (จาก v0.1)
`tune_sweep` (สแกนค่าชดเชยการหยุด tau) และ `summarize_tune` (สรุปผลของ sweep) ใช้ได้เหมือนเดิม
