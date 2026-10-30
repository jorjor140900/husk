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

## Runtime observed-return transport and deskew

`/velodyne_points` is now supported as an **opt-in** PointCloud2 transport:
ROS1 `cloud_json` preserves the header, fields, strides, endianness and every data
byte in bounded base64. The telemetry worker rate-limits the latest cloud to 5 Hz;
the local command watchdog never performs cloud serialization. The default topic
is empty, so existing deployments do not acquire extra cloud/network load.

Update/export/build Jazzy and copy the newly exported ROS1 package to
`~/husky_ws/src/husky_vision_bridge` on the robot, then `catkin_make --pkg husky_vision_bridge`.
Use the existing no-joystick **preview** profile:

```bash
roslaunch husky_vision_bridge bridge.launch \
  profile_path:="$HOME/husky-nojoy-preview.json" \
  odom_topic:=/odometry/filtered scan_topic:=/scan base_frame:=base_link \
  cloud_topic:=/velodyne_points cloud_max_fps:=5.0 \
  allow_robot_output:=false
```

After updating, restart the gateway and the planning-only navigation so both
load the new bridge code. Keep `allow_motion=false` and the SSH tunnel running. In a
Jazzy terminal sourced with ROS/venv/workspace and Cyclone DDS:

```bash
ros2 run husky_vision vlp16_observer
```

Inspect:

```bash
ros2 topic echo /husky_nav/vlp16_status --once --full-length
```

Default `input_mode=inspect` checks raw layout, signed point times and source
freshness and reports `source_timing_mode_required`; it does not deskew or publish
geometry until the input timing convention has been verified. Inspect the robot's
`rosparam get /velodyne_nodelet_manager_transform`, especially `fixed_frame` and
`target_frame`, to avoid applying motion compensation twice. After confirming
that the input consists of raw sensor returns at their per-point times, run:

```bash
ros2 run husky_vision vlp16_observer --ros-args -p input_mode:=raw_timed
```

In this explicit mode, the observer uses the measured sensor-to-base TF at the cloud stamp and waits
for odometry brackets over **all** signed point times. It never extrapolates or
replaces the source stamp with receipt time. Old/malformed clouds, missing TF,
nonplanar odometry, missing brackets and odometry gaps remain invalid. It publishes
only deskewed measured returns at `/husky_nav/vlp16_observed_points` in `base_link`.
In RViz a PointCloud2 display may use this topic with a short decay time; always
check the status as well. No points are synthesized for missing rays.

`valid=true` and `deskewed_observed_returns_only` mean that transport and geometric
motion compensation succeeded. **They do not mean a safe command is allowed.**
Ground/body filtering, obstacle-height coverage, the near-field blind region and
physical stopping checks are still required before a cloud-based control filter
can replace the rejected planar scan. The observer has no velocity publisher,
permission publisher or arm client. It deliberately leaves the existing final
filter and the pending field measurements unchanged. This step does not repair
`invalid_scan_metadata` or enable autonomous motion.

Live transform-node configuration supplied on 2026-10-30:
`fixed_frame=''`, `target_frame=''`, `min_range=0.4`, `max_range=130.0`,
`organize_cloud=false`, VLP16db calibration, full horizontal view.
This configuration supports choosing `input_mode=raw_timed` for this deployment;
recheck it after driver changes. These cloud-source limits differ from the
LaserScan's reported 0..200 m and do not establish free-space/low-obstacle coverage.

## Additional stationary bag received 2026-10-30

`husky-vlp16-20261029-232306.bag`, SHA-256
`b25abb043e86f151dd10c2dd32ddc9ef3a84c2a63c34aee0b6145a8c5c25c01c`:
34 clouds/scans, 171 odometry messages and static TF. No packet topic was requested
for this recording; the inspection tool now treats packets as optional and skips
packet-relative checks, while retaining signed PointCloud2 times.

32 clouds have complete odometry brackets; two boundary clouds are rejected.
Odometry pose extent is zero, so this recording does not validate deskew during
real motion. Scan metadata is rejected in all 34 scans; all match ring 8.

Declared TF places base_footprint at base Z=-.13228 m and the sensor at
base XYZ=(-.2317,0,.88155), identity rotation. On the assumed flat plane this is
1.01383 m sensor height. The lowest ring is -15 degrees. Along the forward
centerline its height above the declared plane is approximately:

| Forward X from base (m) | Lowest-beam height (m) |
| --- | --- |
| 0.5 | .818 |
| 1.0 | .684 |
| 1.5 | .550 |
| 2.0 | .416 |
| 2.5 | .282 |
| 3.0 | .148 |

In the observed 1.2 m wide front strip there are no returns before X=1.5 m,
and no returns within 5 cm of the declared floor below X=3 m. This is **not** a
free-space measurement. Returns at X=1.5..2.5 m have heights roughly .31..63 m;
without synchronized imagery they cannot be identified as particular objects.
Ground-height candidates occur around X=3..5 m, about 2..3 cm below the declared
plane. No ground-class or passability decision is made from height alone.

No points project inside the previously stated footprint XY rectangle
X=[-.56,.50], Y=[-.33,.33]. That absence cannot identify a 3-D self-filter model
or prove that the body is filtered correctly. The dimensions used are prior
user measurements, not a new measurement from this bag.

Changing scan limits or dropping floor/body points cannot restore missing
near-floor observations. A cloud-based stop filter still needs an explicit
near-field coverage strategy and physical validation; obstacle classification
alone must not permit commands into unobserved space. The recorded stationary
scene does not validate stopping distances, terrain or camera calibration.
