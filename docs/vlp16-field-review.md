# ผลตรวจ VLP16 ของ cpr-a200-1068

ตรวจไฟล์ `husky-vlp16-check_2026-10-26-00-33-44.bag` ที่ผู้ใช้บันทึกบนหุ่นจริง
SHA-256: `1acf2891b337885f85d60f59733a74030fdc933766e09bd8874ae7139394897e`
ช่วงบันทึกประมาณ 2.7 วินาที มี 27 LaserScan/PointCloud2/VelodyneScan และ 137 odometry
หุ่นในข้อมูลนี้อยู่นิ่ง: pose extent และ v/omega ที่รายงานเป็นศูนย์

## ข้อมูลที่ยืนยันได้

| รายการ | ผลจาก bag |
| --- | --- |
| Odometry | `/odometry/filtered`, `odom -> base_link`, ประมาณ 50 Hz |
| LiDAR | `/scan`, `/velodyne_points`, `/velodyne_packets`, ประมาณ 9.9 Hz |
| LiDAR frame | `velodyne` |
| Static TF จาก base_link | X=-0.2317 m, Y=0, Z=0.88155 m, rotation identity |
| PointCloud2 | x/y/z/intensity FLOAT32, ring UINT16, time FLOAT32, point_step=22 |
| จำนวนจุดต่อ cloud | 25,382–25,612 จุด ครบ 16 rings |
| เวลารายจุดเทียบกับ header | -99.540 ถึง +1.306 ms |
| ช่วงการเก็บจุดต่อ cloud | ประมาณ 100.7–100.8 ms |
| Stamp ของ cloud | ตรงกับ stamp ของ packet สุดท้าย ไม่ใช่จุดแรก |

TF ข้างต้นเป็นค่าที่ระบบหุ่นประกาศ ต้องตรวจเทียบการติดตั้งจริง
Z=0.88155 วัดจาก base_link ไม่ใช่พื้น ไม่ใช้แทนความสูงกล้อง
อายุข้อมูล ณ เวลาเขียน bag ของ scan มี median ประมาณ 5.55 ms; ค่านี้ไม่ใช่ latency รวมข้าม bridge/AI

## เหตุที่ยังใช้ /scan ตรงกับตัวกรองไม่ได้

1. ทั้ง 27 scans มี `range_min=0` และ `angle_max` ไม่ตรงกับจำนวน bin/step
   จำนวน 897, increment≈0.007 rad ทำให้ bin สุดท้ายเป็น 3.130407 rad แต่รายงาน angle_max≈3.141593
   ตัวกรองเดิมจึงปฏิเสธด้วย `invalid_scan_metadata` ถูกต้องตามสัญญาของมัน
2. สร้าง scan ซ้ำด้วยวงที่ 8 และวิธีเขียนค่าของ driver แล้วตรงกับข้อมูลจริงครบทั้ง 27 เฟรม
   วงนี้เงย +1° จึงไม่ได้ครอบคลุมสิ่งกีดขวางเตี้ยใกล้หุ่น
3. Driver เขียนค่าจุดสุดท้ายทับ bin เดิม ไม่ได้เลือกจุดใกล้ที่สุด
   พบ 4,872 bin-occurrences ที่ค่าท้ายไกลกว่าจุดใกล้ที่สุดใน bin เดียวกันเกิน 1 cm
   ค่าส่วนต่างเป็นข้อมูลของ *bin ที่รวมหลายมุมย่อย* ไม่ใช่ข้อสรุปว่า sensor วัดระยะผิด
4. มี bin ที่ไม่มี return 31–48 ช่องต่อ scan; ต้องรักษาความหมายว่าไม่มีหลักฐาน
   ไม่ถือว่าเป็นพื้นที่ว่างถึง 200 m เพียงเพราะค่าเป็น +inf
5. `scan_time=0` และ `time_increment=0` ไม่ได้พิสูจน์ว่า instantaneous/deskewed
   raw packets กับ field time ของ cloud แสดงว่ามีช่วง acquisition ประมาณ 101 ms

ตรวจพฤติกรรมประกอบกับ [official velodyne_laserscan 1.7.0 source](https://github.com/ros-drivers/velodyne/blob/1.7.0/velodyne_laserscan/src/velodyne_laserscan.cpp)
โดยใช้การเทียบข้อมูลจริงยืนยันพฤติกรรม ไม่ได้สมมติว่าหุ่นติดตั้ง package version นี้

## โค้ดที่เพิ่มและขอบเขต

`husky_drivable_area/vlp16.py` มีตัวอ่าน PointCloud2 ที่คง signed time, ring และ xyz
รองรับ byte order/row padding และ deskew ตาม planar odometry ที่ interpolate ณ เวลาของแต่ละจุด
ใช้ static lidar-to-base ที่ให้มาอย่างชัดเจน ไม่แทน TF ที่ขาดด้วย identity
ต้องมี odometry คร่อมช่วง acquisition ทั้งหมด รวมจุดที่อยู่หลัง header เล็กน้อย
ถ้า odometry ขาด มีช่องว่างเกินเกณฑ์ หรือ layout/เวลาผิด จะปฏิเสธข้อมูล

ทดลองกับ bag นี้ประมวลผลได้ 26 clouds; cloud แรกถูกปฏิเสธเพราะข้อมูล odometry ต้น bag
ไม่คร่อม acquisition ก่อน header ประมาณ 100 ms ไม่ extrapolate หรือสมมติว่าหุ่นนิ่งแทนข้อมูลที่ขาด
ผล deskew ของ 26 clouds ไม่เปลี่ยนจุด เพราะ odometry ใน bag เป็นศูนย์ทั้งหมด
unit tests แยกทดสอบการเคลื่อนที่ตรง การหมุนผ่าน ±pi, sensor offset และ input ที่ขาด
แต่ยังไม่ใช่ผลยืนยัน deskew/odometry ขณะหุ่นจริงเคลื่อนที่

**ยังไม่ได้ต่อ adapter นี้เข้าทางเดินคำสั่งของ robot_pipeline**
โมดูลนี้คืนเฉพาะ measured returns ใน base ที่เวลา header ไม่มีการสร้าง LaserScan ที่ผ่าน gate
ไม่มี height/ground/self filtering หรือการรับรอง free-space/coverage และไม่ publish คำสั่ง
ตัวกรองเดิมยังคงปฏิเสธ /scan นี้ ไม่เปลี่ยน metadata เพียงเพื่อให้ผ่าน
ก่อนใช้งานเคลื่อนที่ต้องทำ runtime integration, ตรวจเวลา/TF ขณะเคลื่อนที่ และกำหนด geometry/coverage
จากการติดตั้งจริง รวมถึงการมองไม่เห็นสิ่งกีดขวางเตี้ย/ขอบตกที่ LiDAR ติดสูงอาจตรวจไม่พบ

## ตรวจ bag ซ้ำแบบ offline

รันจากโฟลเดอร์ Git ใน environment สำหรับวิเคราะห์ ไม่ต้องติดตั้ง dependency ชุดนี้บนหุ่น:

```bash
python -m pip install -r requirements-lidar-review.txt
python -m tools.inspect_vlp16_bag \
  --bag /path/to/husky-vlp16-check_2026-10-26-00-33-44.bag \
  --output outputs/vlp16-review-01 \
  --plots
```

ต้องใช้ output directory ใหม่ เครื่องมือส่งออก report.json และ vlp16-review.png
ไม่ต้องเปิด ROS master, ไม่ส่งข้อมูลเข้าหุ่น และไม่เปลี่ยน permission

กล้องของงานนี้ผู้ใช้แจ้งว่าติดหุ่นแล้วและเสียบ USB กับคอม ROS1
ต้องตรวจ device/driver, ใช้ camera.source=ros1_compressed และวัด mount ใหม่ก่อนเชื่อม image-to-metric path
