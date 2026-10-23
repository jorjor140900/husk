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

## 4. โค้ดที่เตรียมไว้สำหรับต่อกับหุ่นภายหลัง

| ส่วน | หน้าที่ |
| --- | --- |
| `husky_vision/control.py` | Pure Pursuit + command limits แบบไม่พึ่ง ROS |
| `husky_vision/controller_node.py` | รับ Path, odometry, TF และ permission heartbeat; ส่ง Twist/สถานะ |
| `husky_vision/control_simulator.py` | เส้นสังเคราะห์และหุ่นจำลองวงจรปิด |
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
