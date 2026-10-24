# ทดลอง controller ใน ROS 2 Jazzy โดยยังไม่ต้องมีหุ่นหรือออกพื้นที่

พักการตรวจ calibration/ระยะภาคสนามไว้ก่อนได้ ไฟล์ `camera.yaml`, `camera-v2.yaml`
และ `mount.json` เก็บไว้ตามเดิม ชุดทดลองนี้ใช้ **เส้นทางสังเคราะห์ + หุ่นจำลองทางคณิตศาสตร์**
ไม่มีการเปิดกล้อง โหลด SegFormer หรือเชื่อม ROS 1/bridge

เป้าหมายคือทดสอบ **เส้นหน่วยเมตร → controller → linear.x/angular.z → odometry → controller**
ในวงจรปิด รวมการหยุดเมื่อข้อมูลขาด ยังไม่ได้ยืนยันว่าพร้อมขับ Husky จริง

## 1. อัปเดต workspace เดิม

เปิด Terminal ที่โฟลเดอร์ Git เดิมซึ่งมี `RoadDetection.py`:

```bash
git pull --ff-only origin work
python3 tools/create_ros2_workspace.py --workspace ~/husky_vision_ws --update-code
```

สำรอง source ที่มีอยู่ก่อนใน `.source-backups/<เวลา>/` ไม่แก้โมเดล, detector,
`config.example.json` หรือ calibration ที่เก็บใน `~/husky_vision_ws/camera_calibration/`
ไฟล์ ROS, metadata, launch และไฟล์ประกอบใน source package จะอัปเดต

เปิด Terminal ใหม่เพื่อ build:

```bash
cd ~/husky_vision_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
python /usr/bin/colcon build --symlink-install --packages-select husky_vision
source install/local_setup.bash
```

ใช้ `local_setup.bash` ตามเดิม ถ้ายังไม่ได้ติดตั้ง RViz ให้รัน `sudo apt install ros-jazzy-rviz2`

## 2. เปิดการทดลองด้วยคำสั่งเดียว

หยุด launch กล้อง/projection เดิมเพื่อลดภาระ CPU แล้วรันใน Terminal ที่ build ด้านบน:

```bash
QT_QPA_PLATFORM=xcb ros2 launch husky_vision control_demo.launch.py
```

RViz จะแสดง:

- **เส้นเหลือง:** reference สังเคราะห์ในหน่วยเมตร
- **กล่องน้ำเงิน + ลูกศรขาว:** ตำแหน่งและทิศหุ่นจำลอง
- **เส้นฟ้า:** เส้นทางที่หุ่นจำลองเคลื่อนที่จริง
- ตารางพื้นช่องละ **0.5 m**; Fixed Frame คือ `husky_demo/odom`

เริ่มต้นหุ่นอยู่เยื้องเส้น 0.45 m ตัว controller จะเลี้ยวเข้าหาเส้น ตามทางโค้ง
และหยุดเมื่อถึงปลายทาง ความเร็วเป้าหมาย 0.2 m/s เส้นยาวประมาณ 8 m จึงใช้เวลาประมาณหนึ่งนาที
ตัวเลขมิติกล่องเป็นภาพประกอบ ไม่ใช่ footprint หรือโมเดลพลวัตของ Husky A200
Pure Pursuit ใช้ตำแหน่ง odometry ปัจจุบันเลือกจุด lookahead บนเส้น แล้วคำนวณความโค้ง
มีการจำกัดความเร็ว อัตราหมุน และอัตราเปลี่ยนคำสั่งระหว่าง tracking

เปลี่ยนฉากได้ โดยหยุด launch เดิมด้วย Ctrl+C ก่อน:

```bash
QT_QPA_PLATFORM=xcb ros2 launch husky_vision control_demo.launch.py scenario:=straight
QT_QPA_PLATFORM=xcb ros2 launch husky_vision control_demo.launch.py scenario:=obstacle
```

รัน **ทีละคำสั่ง**; อย่าเปิดหลาย demo ซ้อนใน namespace เดียว
ฉาก `obstacle` มีทรงกระบอกแดงที่ X=3 m และ gate จำลองจะสั่งหยุดก่อนถึงวัตถุ
gate นี้อ่านตำแหน่งจริงของวัตถุใน simulation ไม่ได้จำลอง LiDAR และยังไม่มีการวางแผนหลบ
ใช้ `rviz:=false` ได้หากต้องการเฉพาะ nodes โดยไม่มี GUI

## 3. ดูคำสั่งและทดสอบการหยุด

Terminal อีกหน้า:

```bash
source /opt/ros/jazzy/setup.bash
source ~/husky_vision_ws/install/local_setup.bash
ros2 topic echo /husky_demo/control/status --once
```

`linear_x` หน่วย m/s, `angular_z` หน่วย rad/s; บวกคือเลี้ยวซ้าย
`tracking` คือกำลังตามเส้น, `goal_reached` คือถึงปลายทาง
คำสั่งควบคุมจริงของ demo อยู่ที่ `/husky_demo/control/cmd_vel` ชนิด `geometry_msgs/Twist`
ไม่มีการส่งไป `/cmd_vel` ของหุ่นจริง

**ทดสอบ software stop:**

```bash
ros2 service call /husky_demo/simulation/set_stop std_srvs/srv/SetBool '{data: true}'
```

กล่องต้องหยุด และ status เป็น `motion_not_permitted` พร้อม v=0, omega=0
ปล่อย stop เพื่อให้ตามเส้นต่อ:

```bash
ros2 service call /husky_demo/simulation/set_stop std_srvs/srv/SetBool '{data: false}'
```

**ทดสอบ reference หาย:**

```bash
ros2 param set /husky_demo/control_simulator publish_reference false
```

ภายในประมาณ 0.5 วินาที + หนึ่งรอบ control คำสั่งต้องเป็นศูนย์และ status เป็น `stale_path`
เปิดกลับด้วย:

```bash
ros2 param set /husky_demo/control_simulator publish_reference true
```

ทดสอบ `publish_odometry false` หรือ `publish_permission false` แบบเดียวกันได้
ต้องหยุดด้วยเหตุผล `stale_odometry` หรือ `permission_timeout` ตามลำดับ ภายในประมาณ 0.3 วินาที + หนึ่งรอบ control
เปิดกลับเป็น `true` เพื่อทดลองต่อ ระบบจะกลับมาตามเส้นเมื่อข้อมูลใหม่ครบและ gate อนุญาต

รีเซ็ตตำแหน่งหุ่นจำลอง:

```bash
ros2 service call /husky_demo/simulation/reset std_srvs/srv/Trigger '{}'
```

reset คงค่า stop และตัวเลือกปิด/เปิด stream เดิมไว้ หากยังหยุดให้ตรวจค่าเหล่านี้
software stop นี้เป็นการทดสอบโปรแกรม ไม่แทน hardware E-stop

## ทดสอบอัตโนมัติและเก็บผลก่อนปรับ controller

หลังเปิด demo ได้แล้ว ให้หยุด demo เดิมด้วย Ctrl+C เพื่อลดภาระ CPU
อัปเดต source ด้วย `--update-code` และ build ตามขั้น 1 แล้วเปิด Terminal:

```bash
cd ~/husky_vision_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/local_setup.bash
ros2 run husky_vision control_self_test
```

ไม่ต้องเปิด RViz, webcam หรือ launch อื่น โปรแกรมเปิด controller กับ simulator ให้เอง
ใน namespace สุ่ม แล้วปิดเมื่อจบ ใช้เวลาประมาณ 10–20 วินาทีด้วย config เริ่มต้น
อ่านค่าจาก `config/controller.yaml` ที่ติดตั้งไว้ โดยบังคับ topic/frame ให้เป็นของการทดสอบเอง
และเริ่ม disabled เพื่อทดสอบการเปิด controller

ผลสำเร็จท้าย Terminal จะมี `"result": "passed"` และ `"checks_passed": 15`
พร้อม `Report: .../outputs/control-check/control-<เวลา>-<รหัส>.json`
เก็บไฟล์แยกต่อการรัน ไม่เขียนทับผลเก่า และมีรายงานเมื่อการทดสอบล้มเหลวด้วย

| ค่าในรายงาน | ความหมาย |
| --- | --- |
| `tracking.rmse_m` | RMS ของระยะคลาดเคลื่อนตลอดช่วงสังเกต รวมช่วงแรกที่เริ่มเยื้องเส้น |
| `tracking.tail_mean_m` | ค่าเฉลี่ยระยะคลาดเคลื่อนใน 10% สุดท้ายของตัวอย่าง |
| `tracking.distance_forward_m` | การเปลี่ยน X ของหุ่นจำลองระหว่างวัด ไม่ใช่ระยะสะสมตามทาง |
| `stop_observations.*.request_to_zero_seconds` | เวลาจากส่งคำขอหยุด/ตัดข้อมูลจนผู้ทดสอบรับคำสั่งศูนย์กับ status |
| `checks` | เงื่อนไขที่ผ่าน เช่นข้อมูลขาด, TF ณ capture time, goal stop และ disarm |
| `failed_check`, `error` | ขั้นที่ล้มเหลวและรายละเอียด เมื่อ `result=failed` |

เกณฑ์ demo เริ่มต้นคือมีตัวอย่างอย่างน้อย 20 ตัวอย่าง, เดินหน้าเกิน 0.02 m,
ค่าเฉลี่ย error ช่วงท้ายไม่เกิน **0.15 m** และไม่ขาดข้อมูลระหว่าง tracking
เก็บ tracking 8 วินาทีโดยค่าเริ่มต้น ยังไม่ใช่การวิ่งครบเส้น 8 m
ข้อมูล error มาจาก controller เทียบ reference สังเคราะห์กับ odometry จำลอง
เวลา request→zero ไม่ใช่ระยะเบรกหรือเวลาที่หุ่นจริงหยุด
งบเวลาทดสอบกรณีตัดข้อมูลคือ timeout ที่ตั้งไว้ + เผื่อ scheduler อย่างน้อย 0.25 s
ทั้งหมดเป็นเกณฑ์ตรวจซอฟต์แวร์ใน demo ไม่ใช่เกณฑ์ยืนยันความพร้อมของ Husky จริง

ตัวอย่างผลที่ตรวจใน cloud: 160 ตัวอย่างใน 8 วินาที, RMSE ประมาณ 0.209 m
โดยเริ่มห่างเส้นประมาณ 0.415 m และ tail mean ประมาณ 0.011 m
ค่าเริ่มต้นสูงเพราะจงใจเริ่มเยื้องเส้นเพื่อทดสอบการเลี้ยวกลับเข้า reference
อย่าใช้ตัวเลข simulation นี้อ้างเป็นความแม่นยำของหุ่นจริง

หากปรับ gains ให้เก็บ config ไว้นอก source ที่ updater จะเปลี่ยน แล้วทดสอบเทียบด้วย:

```bash
cp -n src/husky_vision/config/controller.yaml controller-tuning.yaml
nano controller-tuning.yaml
ros2 run husky_vision control_self_test --config "$HOME/husky_vision_ws/controller-tuning.yaml"
```

ใช้ `--tracking-seconds 45` เพื่อสังเกตนานขึ้นได้ หรือ `--output-dir <โฟลเดอร์>` เพื่อเลือกที่เก็บรายงาน
เริ่มด้วย config เดิมก่อน ไม่ต้องเพิ่มความเร็วเพื่อให้ error ลดเร็วขึ้น
หลังการทดสอบผ่าน ขั้นพัฒนาต่อคือ metric footprint/collision gate กับ LiDAR
และการเชื่อม TF/perception โดยคงการตรวจระยะจริงไว้ก่อนต่อ hardware

## หลังผ่าน 15 checks: ทดลอง LiDAR กับพื้นที่กวาดผ่านของตัวหุ่น

ชุดใหม่นี้เปิด **controller → requested Twist → LaserScan filter → actual Twist → หุ่นจำลอง**
ตัวกรองอ่าน scan และ odometry; ไม่อ่านตำแหน่งทรงกระบอกจาก simulator เพื่อสั่งหยุด
simulation ยังใช้ ray/circle intersections สร้าง scan จากวัตถุที่รู้ตำแหน่งตามปกติของ sensor simulator
ชุด `control_demo`/`control_self_test` เดิมยังใช้เป็น baseline ได้ตามเดิม

อัปเดต source ด้วย `--update-code` แล้ว build ตามขั้น 1 หยุด demo เก่าก่อนเปิด:

```bash
cd ~/husky_vision_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/local_setup.bash
QT_QPA_PLATFORM=xcb ros2 launch husky_vision lidar_demo.launch.py
```

RViz ใช้ Fixed Frame `husky_lidar_demo/odom` พร้อมกล่องน้ำเงิน, เส้นเหลือง,
ทรงกระบอกแดงและจุดจาก LiDAR จำลอง กรอบเขียวคือพื้นที่ตามแบบจำลองที่จะตรวจ
เมื่อพบจุด scan ภายในพื้นที่นั้น กรอบเป็นแดงและหุ่นหยุดก่อนถึงทรงกระบอก
แสดง scan เก่าค้างไว้ไม่เกินประมาณ 0.3 s

Terminal อีกหน้า:

```bash
source /opt/ros/jazzy/setup.bash
source ~/husky_vision_ws/install/local_setup.bash
ros2 topic echo /husky_lidar_demo/safety/status --once
```

`clear` หมายถึงคำสั่งผ่านตัวกรองตามโมเดลนี้; `obstacle_in_sweep` หมายถึงมี return ในพื้นที่ที่ตรวจ
`control/requested_cmd_vel` คือคำสั่งก่อนกรอง ส่วน `control/cmd_vel` คือคำสั่งที่ส่งเข้าหุ่นจำลองจริง
ทั้งสองอยู่ใต้ `/husky_lidar_demo/` ไม่มีการส่งไป `/cmd_vel` ของหุ่นจริง

ทดสอบเอาวัตถุออก โดยหุ่นจะเคลื่อนที่ต่อเมื่อ scan ใหม่ผ่านและข้อมูลครบ:

```bash
ros2 param set /husky_lidar_demo/control_simulator obstacle_enabled false
```

ทดสอบ scan หาย:

```bash
ros2 param set /husky_lidar_demo/control_simulator publish_scan false
```

ภายในประมาณ 0.3 s + หนึ่งรอบ filter ต้องเห็น `stale_scan` และความเร็วศูนย์
เปิดกลับด้วย `publish_scan true` และใส่วัตถุกลับด้วย `obstacle_enabled true`
ตัวกรองไม่ latch การหยุด: เมื่อข้อมูลใหม่ผ่านต่อเนื่อง 0.5 s และอนุญาตจะกลับมาวิ่งเอง
ระหว่างรอจะเป็น `waiting_for_clear` จากนั้นเพิ่มความเร็วแบบจำกัด acceleration
เหตุผลหยุดหรือ request ที่เป็นศูนย์ยังส่งศูนย์ทันที ไม่รอ ramp
ใช้ software stop/reset เดิมโดยเปลี่ยน prefix เป็น `/husky_lidar_demo/`
ทดลองไม่มีวัตถุด้วย `scenario:=curve` หรือ `scenario:=straight` ได้ โดยเปิดทีละ launch

### ตัวกรองตรวจอะไร

- ใช้ rectangle ที่มี front/rear/half-width + margin จาก `config/laser_safety.yaml`
  ค่า 0.45/0.45/0.30 m เป็น **ขนาดกล่อง demo** ต้องวัด Husky พร้อม payload ก่อนใช้จริง
- ตรวจ motion ที่ร้องขอ, motion หลังจำกัด acceleration และ motion ที่วัดได้จาก odometry แม้คำสั่งใหม่เป็นศูนย์
  ทำนายแต่ละแนวหยุดด้วยความโค้งคงที่ ช่วง reaction และการลด v/omega ลงพร้อมกัน
- เพิ่มอายุ scan/odometry ในเวลาตอบสนองที่ใช้คำนวณ ขยับกรอบทีละไม่เกิน 0.01 m
  ตามขอบเขตความเร็วของจุดบนตัวหุ่น และ inflate กรอบด้วยระยะ sampling อีกชั้น
- แปลง scan ไป odom ด้วย TF ณ scan timestamp แล้วแปลงกลับเข้ากรอบตัวหุ่นด้วย odometry ล่าสุด
  หาก TF ไม่มี, ข้อมูลเก่า, หาย หรือไม่ถูกต้อง จะส่งศูนย์
- ต้องเห็น scan รอบตัวครบ 360° มี angular increment ไม่เกิน 0.02 rad
  `+inf` หมายถึง no-return ถึง range_max ตามสัญญาของ sensor; NaN, -inf, ระยะผิดช่วงเป็น unknown และหยุด
- ต้องให้กรอบที่ตรวจทั้งกรอบอยู่ใน range_max ของ sensor และรับเฉพาะ scan instantaneous/deskewed
  `time_increment != 0` จะหยุดด้วย `scan_requires_deskew` ต้องทำ deskew ก่อนต่อ LiDAR จริงที่หมุนเก็บข้อมูล

นี่เป็น **ต้นแบบหยุดจาก planar scan** ไม่ใช่ collision avoidance planner หรือระบบ safety ที่รับรองแล้ว
ยังไม่ตรวจ metric drivable mask, หลุม/ขอบตก, ของที่อยู่เหนือ/ใต้ระนาบสแกน หรือวัตถุที่ LiDAR ไม่เห็น
แนวหยุดที่ตรวจไม่ครอบคลุมการเปลี่ยนความโค้งทุกแบบของหุ่นจริง; การลื่นไถล การเบรกและ actuator lag ต้องวัดและปรับโมเดล
ต้องรวม drivable map/footprint, deskew, ขอบเขตการรับรู้, TF, hardware watchdog และ E-stop ก่อนต่อฐานหุ่น
ค่า `physical_robot_validated` ใน status จึงยังเป็น false

ตรวจใน official ROS Jazzy container แล้ว: unit tests รวม 100 รายการ,
ข้อมูล LaserScan/odometry/request/permission ขาดหรือผิดผ่านการทดสอบด้วย ROS messages จริง,
และเปิด launch ครบสาม process แบบไม่มี GUI ทั้งฉาก obstacle กับ curve
ฉาก obstacle หยุดนิ่งที่ X ประมาณ 1.967 m ก่อนวัตถุ X=3 m โดย requested command ยังเป็นบวกแต่ filtered command เป็นศูนย์
การเอาวัตถุออก, scan หาย/กลับมา และ curve tracking ผ่านตัวกรองทำงานได้
ชุด `control_self_test` เดิมยังผ่าน 15 checks ค่าเหล่านี้เป็นผล simulation
ไม่ได้ทดสอบหน้าต่าง RViz บน desktop หรืออุปกรณ์ LiDAR/หุ่นจริง

## เชื่อมกล้องจริงกับ preview ของ controller ระหว่างรอลงพื้นที่

หลังทดสอบ controller/sensor จำลองแล้ว ให้กลับมาตรวจส่วนงานภาพด้วย
`vision_preview.launch.py`: **webcam → SegFormer → pixel reference → ground projection → Pure Pursuit candidate**
เปิดภาพ overlay กับ RViz ให้ในคำสั่งเดียว โดยใช้โมเดลเดิม ไม่ต้อง train ใหม่

นี่เป็นการดูค่าที่กฎควบคุมคำนวณได้ **เมื่อสมมติว่าจุดกำเนิดบนพื้นอยู่นิ่ง**
ไม่มี odometry จริงและไม่มีหุ่นจำลองวิ่งตามภาพ ไม่มีการสร้าง `odom → base_link` ปลอม
node นี้ส่งเฉพาะ Path, MarkerArray และ JSON ไม่ประกาศ publisher ชนิด Twist หรือช่อง cmd_vel
ไม่มี service สำหรับ arm ให้สั่งมอเตอร์จาก preview นี้

### เตรียมกล้องในห้อง

ยึดกล้องให้นิ่ง มองพื้นราบ และตรวจว่า `camera_calibration/mount.json` เป็นความสูง/มุมก้มของตำแหน่งปัจจุบัน
คงความละเอียด capture 640×480 และ focus/zoom ตามที่คาลิเบรตไว้
กำหนด origin บนพื้นใต้เลนส์, แกน X ตามทิศกล้อง, ปรับกล้องให้ระดับเพื่อใช้ roll/yaw=0
ถ้าการติดตั้งเปลี่ยน ให้วัด mount ใหม่; เก็บค่าของการติดตั้งคนละครั้งแยกชื่อไฟล์ได้
สำหรับขั้นนี้ยังไม่ต้องรู้การติดตั้งกล้องบน Husky จริง
`pitch_down_deg` วัดก้มลงจากแนวราบ: `0` คือมองระดับ, `70` คือก้มลง 70°
เช่น กล้องสูง 77 cm และก้มประมาณ 70° ต้องใช้ `height_m: 0.77`, `pitch_down_deg: 70.0`
ห้ามคงมุมเป็น 0 เมื่อกล้องก้มอยู่ เพราะระยะฉายลงพื้นจะผิดและอาจขึ้น `too_far_from_path`
แก้เฉพาะ mount แล้ว restart launch ได้ ไม่ต้องคาลิเบรต intrinsics หรือ train ใหม่
มุมประมาณใช้ตรวจ preview ได้ แต่ต้องเทียบระยะบนพื้นที่วัดจริงก่อนใช้ควบคุม

อัปเดต `--update-code` แล้ว build ตามขั้น 1 หยุด launch กล้อง/projection/demo เดิมก่อน
เพื่อไม่เปิด webcam/detector หรือ TF ของกล้องซ้อนกัน แล้วรัน:

```bash
cd ~/husky_vision_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/local_setup.bash
QT_QPA_PLATFORM=xcb ros2 launch husky_vision vision_preview.launch.py \
  camera_index:=2 \
  model_path:="$HOME/husky_vision_ws/models/husky-segformer" \
  camera_calibration:="$HOME/husky_vision_ws/camera_calibration/camera-v2.yaml" \
  camera_mount:="$HOME/husky_vision_ws/camera_calibration/mount.json"
```

ถ้าใช้ calibration ชื่อ `camera.yaml` ให้เปลี่ยนเฉพาะชื่อไฟล์ให้ตรง
ค่าเริ่มต้นคือ CPU, model input 320×192, 2 threads; capture คง 640×480
`show_image:=false` หรือ `rviz:=false` ใช้ปิดแต่ละหน้าต่างได้
`use_webcam:=false image_topic:=/your/image_raw` ใช้รับภาพจาก publisher ภายนอก โดย launch ยังเปิด detector ให้
ห้ามนำ calibration ของ webcam ไปใช้ฉายคลิปจากกล้องคนละตัวโดยถือว่าระยะถูกต้อง

### ดูอะไรบนหน้าจอ

- ภาพ overlay เดิมแสดง drivable mask และ reference จาก AI
- เส้นใน RViz คือ pixel reference ที่ฉายลงพื้นด้วย calibration/mount ที่ให้ไว้
  สีเหลืองคืออายุผ่านเกณฑ์ controller; สีส้มคือผลยังแสดงเพื่อตรวจงาน แต่อายุเกินเกณฑ์ควบคุมแล้ว
- ลูกศรน้ำเงินคือทิศ X ของ **origin ที่สมมติ** ไม่ใช่ตำแหน่งหุ่นจริง
- ลูกศรฟ้ากับจุดเหลืองคือทิศไปยัง lookahead ที่คำนวณได้
- ข้อความ `candidate v` หน่วย m/s และ `omega` หน่วย rad/s เป็นค่าก่อน acceleration limits
  omega บวกคือเลี้ยวซ้าย ลบคือเลี้ยวขวา ไม่ใช่คำสั่งที่ส่งให้หุ่น
- `Candidate BLOCKED` หมายถึงขณะนั้นไม่มี candidate ที่ผ่านเกณฑ์ จะไม่ค้างค่าจากเฟรมเก่า

**การเห็นเส้นกับการผ่านเงื่อนไขควบคุมเป็นคนละสถานะ** โดย preview แสดง geometry
ได้จน source/receipt อายุเกิน `display_timeout_sec` เริ่มต้น 3 s
แต่ candidate/ลูกศร lookahead ถูกล้างเมื่อเกิน `path_timeout_sec` เริ่มต้น 0.5 s ตาม controller
ไม่ขยายเกณฑ์ของ controller เพื่อทำให้ภาพ CPU ที่ช้ากลายเป็นคำสั่งใช้งานได้
ดังนั้นผลภาพอายุ 650–1700 ms จะยังมีเส้นสีส้มและ `Image path: YES`
พร้อม `Candidate BLOCKED: stale_for_control` ไม่ขึ้นว่าตรวจไม่เจอทั้งที่มีผลภาพ
ถ้าหยุดส่งภาพ เส้นเก่าหายเมื่อหมดอายุแสดง; ถ้า detector ส่งผลใหม่ว่าไม่มี path
หรือ calibration/frame ไม่ตรง เส้นเดิมถูกล้างทันที ไม่เก็บเส้นเก่าแทนผลใหม่
ข้อความอัปเดตอายุข้อมูลต่อเนื่องและคงเหตุผลการปฏิเสธของ detector ไว้ ไม่กลบด้วย timeout 0.5 s
ปรับ `display_timeout_sec:=...` ได้เพื่อดูภาพเท่านั้น ต้องไม่น้อยกว่า controller timeout

Terminal อีกหน้าที่ source ROS/workspace แล้ว:

```bash
ros2 topic echo /vision_preview/status --once
```

`candidate_before_acceleration_limits` มี v/omega เมื่อคำนวณได้ หรือเป็น null เมื่อไม่มี candidate
`display_state`, `valid_image_path`, `detection_reason`, `controller_reason` แยกสถานะการเห็นเส้น,
เหตุผลจาก detector และเหตุผลทาง geometry ของ controller; `frame_age_ms` ใช้ timestamp ภาพต้นทางจริง
`path_start_ground_xy_m` คือจุดต้นของเส้นที่ฉายได้ หน่วยเมตร ใช้ตรวจระยะหน้า/ด้านข้าง
`camera_height_m` / `camera_pitch_down_deg` แสดงค่าติดตั้งที่ node ใช้อยู่
`ready_for_control` และ `footprint_checked` เป็น false เสมอ เพราะยังไม่รู้ robot pose/clearance จริง
ถ้าเปลี่ยนชื่อ `ground_frame` ใน mount ให้ปรับ Fixed Frame ของ RViz ให้ตรงกัน

สาเหตุที่พบบ่อย:

| reason | สิ่งที่ได้เรียนรู้ |
| --- | --- |
| `image_path_invalid:*` | detector ยังหา reference ไม่ได้ในพื้นผิว/มุมภาพนี้ ตรวจ overlay และโมเดล |
| `stale_for_control` | มีเส้นสำหรับดู แต่ source/receipt อายุเกินเกณฑ์ควบคุม 0.5 s; ไม่มี candidate |
| `display_expired` | source/receipt อายุเกินเกณฑ์แสดง 3 s; ล้างเส้นและ candidate |
| `waiting_for_detection` | node ยังไม่ได้รับ status จาก detector เลย |
| `stale_or_invalid_source_timestamp` | timestamp เป็นศูนย์/ติดลบ หรืออนาคตเกิน 50 ms; ไม่ฉายเส้น |
| `source_frame_mismatch` | frame ภาพไม่ตรงกับ mount ของกล้อง |
| `invalid_detection_or_calibration_geometry` | อ่าน `detail` เช่น capture resolution ต่างจาก calibration |
| `controller:too_far_from_path` | ระยะสั้นสุดจาก origin ถึงเส้นเกินเกณฑ์ 0.8 m; ตรวจความสูง/มุมก้ม mount ก่อน อาจเป็น blind gap ของกล้อง ไม่ใช่ lateral error เสมอไป |
| `candidate_available` | การคำนวณเรขาคณิตและ Pure Pursuit ได้ค่าในสมมติฐานนี้ ไม่ได้ยืนยันว่าหุ่นผ่านได้ |

ในห้อง หากพื้นไม่เหมือนข้อมูลฝึก อาจไม่พบ drivable area ซึ่งเป็นผลที่ควรเก็บไปวิเคราะห์โมเดล
จุดนี้ใช้สังเกตว่า mask/reference ที่สั่นส่งผลให้ candidate เลี้ยวสลับหรือหยุดอย่างไร และความหน่วงภาพถึงเกณฑ์ controller หรือยัง
ยังไม่ได้ทดสอบวงจรปิดจากภาพบนหุ่นที่เคลื่อนที่ การยืนยันระยะ/footprint, motion compensation, odometry และ LiDAR จริงยังต้องทำภายหลัง

ตรวจ build และการส่งข้อมูลจริงใน official ROS Jazzy แล้ว รวมผล CPU อายุ 655–1671 ms
ที่ยังแสดง geometry แต่ไม่มี candidate, watchdog ล้าง candidate ก่อนล้างเส้น,
resolution/frame ผิด, blind gap และการเก็บเหตุผล detector เมื่อไม่มี path
ตรวจ publisher graph ว่าไม่มี Twist, cmd_vel หรือ Odometry จาก preview
ทดสอบ launch ครบตั้งแต่ Image สังเคราะห์ผ่าน SegFormer test fixture และ detector ไปจนถึง candidate
fixture ใช้เพื่อทดสอบ wiring ไม่ใช่โมเดลที่ train แล้วหรือผลประเมินความแม่นยำภาพ
ตรวจ official image_view ที่ launch เปิดให้ด้วย Qt offscreen แล้วว่า subscribe compressed overlay
และถอดภาพกลับมาได้ โดยไม่มี transport/type mismatch
ยังต้องลอง webcam/โมเดลจริงและ desktop ของผู้ใช้

## 4. โค้ดที่เตรียมไว้สำหรับต่อกับหุ่นภายหลัง

| ส่วน | หน้าที่ |
| --- | --- |
| `husky_vision/control.py` | Pure Pursuit + command limits แบบไม่พึ่ง ROS |
| `husky_vision/controller_node.py` | รับ Path, odometry, TF และ permission heartbeat; ส่ง Twist/สถานะ |
| `husky_vision/control_simulator.py` | เส้นสังเคราะห์และหุ่นจำลองวงจรปิด |
| `husky_vision/control_self_test.py` | วัด tracking และทดสอบ stop ผ่าน ROS พร้อมรายงาน JSON |
| `config/controller.yaml` | lookahead, ความเร็ว, acceleration และ timeout |
| `launch/control_demo.launch.py` | ทดลองครบชุดใน `/husky_demo/` |
| `launch/controller.launch.py` | adapter สำหรับหุ่นจริงใน `/husky_control/` เริ่ม disabled |

ตัวรันทั่วไปเริ่มด้วย `start_enabled=false` ส่วน demo เปิด enabled เฉพาะ namespace ของ simulator
เปิด adapter เพื่อดูสถานะ/คำสั่งที่คำนวณได้ โดย output คงเป็นศูนย์:

```bash
cd ~/husky_vision_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/local_setup.bash
ros2 launch husky_vision controller.launch.py
```

ไม่มี input/TF/gate จริงในตอนนี้จึงเป็นเรื่องปกติที่จะยังไม่คำนวณคำสั่งเคลื่อนที่
`/husky_control/control/cmd_vel_preview` คือ candidate ก่อนการเปิด enabled และ rate limiter
`/husky_control/control/cmd_vel` เป็น output ที่ผ่านการเปิด enabled และข้อจำกัดคำสั่ง
ทั้งสองเป็น Twist; ชื่อ preview ไม่ได้ยืนยันความปลอดภัย
service `control/set_enabled` ชนิด SetBool ใช้เปิด/ปิดตัว controller แต่ไม่ข้ามการตรวจ input/gate
launch สำหรับกล้องหรือ projection จะไม่เปิด controller นี้อัตโนมัติ

### สัญญาของ input

- Path ต้องเป็น `nav_msgs/Path` หน่วยเมตร เรียงตามทิศทางที่จะเคลื่อนที่ และต่อเนื่อง
  ห้ามใช้ `/drivable_area/reference_pixels`; ไม่รองรับ reversing/self-crossing routes หรือ turn-in-place
- Path กับ odometry ต้องมี source timestamps ที่ถูกต้องและนาฬิกาตรงกัน
  ค่าเริ่มต้นยอมรับ path อายุไม่เกิน 0.5 s และ odometry ไม่เกิน 0.3 s
- Odometry ต้องมี parent=`odom_frame`, child=`base_frame` และ quaternion ถูกต้อง
  เป็น feedback ระดับตัวหุ่น ไม่ใช่คำสั่งมอเตอร์ล้อ
- ถ้า Path ไม่อยู่ใน `odom_frame` ต้องมี TF ไป `odom_frame` **ณ timestamp ของภาพต้นทาง**
  node จะตรึงเส้นลง odom ณ เวลานั้น แล้วใช้ odometry ปัจจุบันคำนวณตำแหน่งสัมพัทธ์ใหม่ทุก control tick
  TF ไม่มี/หมดช่วงข้อมูลจะหยุดด้วย `tf_unavailable` ไม่ใช้ transform ล่าสุดแทนย้อนหลัง
- ทุกจุดต้องผ่านการตรวจ drivable/obstacle map กับ footprint และบริเวณที่จะกวาดผ่านตอนเลี้ยวจาก upstream
  controller นี้ไม่ได้ทำ collision checking; path ดูเรียบหรือคำนวณได้ไม่ได้ยืนยันว่าผ่านได้
- `permission_topic` เป็น Bool heartbeat จาก gate อิสระ ต้องเป็น true และเพิ่งรับไม่เกิน 0.3 s
  เมื่อ false/ขาด จะส่งศูนย์ทันทีในรอบ control ถัดไป ไม่รอ ramp deceleration
  heartbeat นี้วัดอายุจาก receipt เพราะ Bool ไม่มี source stamp จึงต้องตรวจความหน่วง gate แยกเมื่อเชื่อมหุ่น

`vision_ground` ของการทดลองกล้องยังไม่ผูกกับตัวหุ่นที่กำลังเคลื่อนที่
ห้ามสร้าง identity static TF ไป `odom` เพื่อให้ error หาย การต่อจริงต้องมี
`odom → base_link` แบบ dynamic และ mount ที่วัดจริงจากตัวหุ่นไปกล้อง/ground frame
พร้อมชดเชยการเคลื่อนที่ระหว่าง timestamp ของภาพกับเวลาสั่ง
projection ปัจจุบันยังรายงาน `ready_for_control=false`/`footprint_checked=false` ตามเดิม

### สิ่งที่ต้องทำเมื่อได้หุ่นและพื้นที่จริง

1. ตรวจ calibration/mount ด้วยระยะจริง และปรับการสร้าง metric drivable/obstacle map ให้รองรับ footprint จริง
2. ตรวจรุ่น ROS 1, ชนิด/ชื่อ topic, TF, odometry และ cmd_vel timeout ของ Husky ก่อนตั้ง bridge
3. ต่อ LiDAR/stop gate อิสระกับ permission; ใช้ E-stop ของหุ่น และ watchdog ที่ฐานหุ่น
   หาก process/network หาย การ publish ศูนย์จาก controller เพียงอย่างเดียวไม่พอ
4. ตรวจความหน่วง perception จริง ถ้าเกิน 0.5 s controller ค่าเริ่มต้นจะหยุด
   ต้องเลือกความเร็ว/ระยะหยุดและปรับปรุง latency จากการวัด ไม่เพิ่ม timeout เพื่อให้วิ่งได้อย่างเดียว
5. ทดสอบทิศทางแกน การหยุด การรับคำสั่ง และ tuning บนหุ่นจริงด้วยความเร็วต่ำในพื้นที่ควบคุม

ล้อและมอเตอร์ใช้ low-level controller ของ Husky ต่อไป ส่วนนี้ส่งเพียง v/omega
ความลื่นไถล, inertia, motor response, พื้นลาดและการเบรกจริงยังไม่ได้จำลอง
การนำทางไปจุดหมาย A→B ยังต้องมี localization/global planning เพิ่มภายหลัง

## การตรวจซอฟต์แวร์

- Unit tests ตรวจแกน/เครื่องหมายเลี้ยว, ความไม่ขึ้นกับ fixed frame, speed/acceleration limits,
  path/pose/quaternion ผิด, freshness และการติดตามทางโค้งจนหยุดที่ปลายทางในเวลาจำลอง
- ทดสอบสร้าง/อัปเดต workspace อิสระและ colcon build ใน official `ros:jazzy-perception`
- `tools/probe_ros2_control.py` เปิด controller กับ simulator เป็นคนละ process ผ่าน DDS จริง
  ใช้ namespace สุ่ม ตรวจ disabled, วงจร odometry, ข้อมูลขาดทั้งสามช่อง, recovery,
  invalid/empty/stale path, TF ณ capture time, goal stop และ disarm
- เปิด `control_demo.launch.py rviz:=false` จริงทั้งฉาก curve และ obstacle
  ตรวจ Path/MarkerArray สำหรับ RViz, การเคลื่อนที่ และไม่พบ publisher ของ `/cmd_vel`
  ฉาก obstacle หยุดที่ประมาณ X=1.808 m, Y=-0.007 m ก่อนทรงกระบอกที่ X=3 m

ผลเหล่านี้ยืนยันพฤติกรรมซอฟต์แวร์ใน simulation ไม่ได้วัด traction/เบรก/ความแม่นยำภาพบนหุ่นจริง
ไม่ได้ทดสอบหน้าต่าง RViz บน desktop ของผู้ใช้จาก cloud
เมื่อต้องการตรวจ DDS ซ้ำ เปิด Terminal ที่ source Jazzy, venv และ workspace แล้วเรียก
`python /path/to/git-repo/tools/probe_ros2_control.py --package husky_vision`
เครื่องมือนี้สร้าง namespace และ TF frames เฉพาะการทดสอบของตน ไม่ส่งคำสั่งไปหุ่นจริง
