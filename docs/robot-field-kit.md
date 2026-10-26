# ชุด ROS1–ROS2 สำหรับวันทดลอง Husky A200

ชุดนี้เตรียมการเชื่อมต่อและการเปิดระบบไว้ล่วงหน้า โดยเก็บค่าหุ่นจริงไว้ใน `robot.json`
โหมดเริ่มต้นรับข้อมูลเท่านั้น ตัว controller, gateway และ receiver เริ่ม disabled/disarmed
การตรวจระยะบนพื้นพักไว้ก่อนได้ แต่ต้องกลับมาตรวจเมื่อยึดกล้องบนหุ่นแล้ว ก่อนเปิดให้เคลื่อนที่

## ส่วนที่เตรียมแล้ว

```text
ROS1 Husky: odom + scan + TF + local permission/deadman + optional compressed camera
    -> robot_gateway -> rosbridge WebSocket -> SSH tunnel
    -> ROS2 Jazzy /husky_link/odom, /scan, /tf, /tf_static

webcam -> SegFormer -> metric Path -> Pure Pursuit
    -> LiDAR filter -> stamped command -> gateway
    -> ROS1 local lease/watchdog -> verified autonomy input of robot velocity mux
```

เลือก application gateway ผ่าน rosbridge เพราะ [official ros1_bridge](https://github.com/ros2/ros1_bridge#supported-ros-and-ubuntu-versions)
ไม่รองรับ Ubuntu 24.04/Jazzy โดยตรง ไม่ต้องย้าย ROS1 มาไว้ในเครื่อง Jazzy
รองรับ odometry, LaserScan, TF, permission Bool, compressed RGB camera และคำสั่งของงานนี้
ไม่มีการ bridge service/parameter/ทุก topic อัตโนมัติ
ROS1 package ใช้ syntax Python 2/3; Noetic เป็นเป้าทดสอบหลัก ส่วน Melodic ต้องตรวจ dependency บนหุ่นอีกครั้ง

Telemetry ห่อใน std_msgs/String โดยเก็บ source timestamp เดิมและเก็บ +inf/-inf/NaN แบบชัดเจน
จึงไม่เปลี่ยนช่องที่ไม่รู้เป็นพื้นที่ว่างจากข้อจำกัด JSON ของ rosbridge
TF static เก็บรวมทุก child และส่ง snapshot แบบ latched/transient-local เมื่อเชื่อมใหม่
ตัดเฉพาะ `/` นำหน้า frame ID ของ ROS1 อย่างสอดคล้องใน odom/TF/ภาพ เพื่อใช้รูปแบบ frame ของ ROS2

## เตรียมในเครื่อง Jazzy ตอนนี้

ในโฟลเดอร์ Git เดิมที่มี RoadDetection.py:

```bash
git pull --ff-only origin work
python3 tools/create_ros2_workspace.py --workspace ~/husky_vision_ws --update-code
```

Terminal ใหม่:

```bash
cd ~/husky_vision_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
python -m pip install -r src/husky_vision/requirements-bridge.txt
python /usr/bin/colcon build --symlink-install --packages-select husky_vision
source install/local_setup.bash
ros2 run husky_vision prepare_robot_kit --workspace ~/husky_vision_ws
ros2 run husky_vision robot_link_self_test
```

ได้ `~/husky_vision_ws/robot/`:

- `README.md`: คู่มือนี้ เปิดอ่านในเครื่องได้
- `robot.json`: ค่า topic/frame/การเชื่อมต่อ/ขนาดหุ่นของจริง จุดที่ยังไม่ทราบคงเป็น null
- `robot-mount.json`: ตำแหน่งกล้องบนหุ่นแยกจาก mount ในห้อง ไม่ใช้ค่าประมาณในห้องแทน
- `ros1_catkin/src/husky_vision_bridge`: package ฝั่งหุ่น พร้อม shared protocol และ watchdog

เครื่องมือไม่เขียนทับ kit ที่มีอยู่แล้ว ไม่เปลี่ยนโมเดลหรือ calibration เดิม
หลังเติมค่าที่วัดแล้ว สำรองโฟลเดอร์นี้ก่อนแก้ config ต่อ
offline self-test ใช้ localhost/namespace/TF frames สุ่มและ receiver จำลอง ไม่ต่อหุ่นหรือส่งคำสั่งไป topic หุ่น
ผลทดสอบยืนยัน software wiring/deadlines ไม่ยืนยันระยะกล้อง การเบรกหรือพื้นที่ที่หุ่นผ่านได้จริง

## วันได้เข้าถึงหุ่น: ติดตั้งฝั่ง ROS1 และเก็บข้อมูลก่อน

คัดลอกโฟลเดอร์ `robot/ros1_catkin` ไปเครื่องหุ่นด้วย USB หรือ scp ตามวิธีเข้าถึงเครื่องของคุณ
ใช้ชื่อ `~/husky_ros1_ws` เพื่อแยกจาก workspace ระบบของหุ่น ไม่แก้ launch หลักหรือ velocity mux เดิม
เปิด terminal ที่ source ROS1/bringup ของหุ่นตามปกติ ตรวจ `echo $ROS_DISTRO` ก่อนติดตั้ง:

```bash
cd ~/husky_ros1_ws
sudo apt install ros-${ROS_DISTRO}-rosbridge-server ros-${ROS_DISTRO}-tf2-msgs
catkin_make
source devel/setup.bash
rosrun husky_vision_bridge robot_inventory.py --output-dir ~/husky_ros1_ws/inventory
```

ROS1 ที่ใช้ Python2 ต้องมี `python-monotonic` ด้วย
รายงาน inventory บันทึกรุ่น ROS, topic/types, odometry frames, scan timing/coverage, TF samples,
Joy buttons/axes, Bool topics และค่า mux/base timeout ที่พบใน parameter ที่เกี่ยวข้อง
ไม่สร้าง publisher คำสั่งหรือแก้พารามิเตอร์หุ่น
เอารายงานกลับมาช่วยเติม `robot.json` โดยตรวจ autonomy input ของ **velocity mux ที่ใช้อยู่จริง**
กับ config bringup และ timeout ของฐานหุ่นด้วย ชื่อ topic อย่างเดียวไม่ยืนยันว่าเป็น input ที่ถูกต้อง

ค่าที่ต้องเติมจากของจริง:

| กลุ่ม | ค่า |
| --- | --- |
| ROS1 | distro, odom_topic, scan_topic, expected_output_topic ของ autonomy/mux |
| TF | odom_frame, base_frame, ระยะ Z จาก base origin ลงพื้น |
| กล้อง | ความสูง/มุม/offset จาก ground frame ของหุ่น และ calibration resolution |
| ตัวหุ่น | front/rear/half-width รวม payload/ส่วนยื่น และ margin |
| การหยุด | base command timeout, mux priority/timeout และ independent permission gate |

เลือก source อนุญาตบนหุ่นอย่างใดอย่างหนึ่ง โดยตรวจความหมายกับอุปกรณ์จริง:

- `robot.ros1_permission_topic`: std_msgs/Bool heartbeat จาก gate บนเครื่องหุ่น ค่า true หมายถึงอนุญาต
  ต้องมีข้อความใหม่ต่อเนื่อง ไม่ใช้ Bool แบบ latched ที่ส่งครั้งเดียว และไม่ต่อ e-stop Bool ที่มีความหมายกลับกันโดยตรง
- หรือ `robot.joy_topic` + `robot.joy_deadman_button`: sensor_msgs/Joy และเลขปุ่มเริ่มที่ 0 ที่วัดแล้วว่าเป็นปุ่มต้องกดค้าง
  โค้ดตรวจทั้ง source stamp และ receipt อายุไม่เกิน 0.3 s; ปล่อยปุ่ม/อุปกรณ์หายจะไม่อนุญาต
  ใช้ปุ่มที่ไม่ทำให้ teleop input แย่ง autonomy ผ่าน mux และตรวจว่า Joy driver ส่งต่อเนื่องขณะกดค้าง
  เช่น autorepeat_rate 10–20 Hz ตาม driver จริง; event-only Joy จะหมดอายุและหยุดตามเกณฑ์
  ถ้าไม่ใช้ source ใดให้คง null; ห้ามกรอกเลขปุ่มเดา

ข้อมูล gate ส่งเป็น `/husky_link/external_permission` เพื่อรวมกับ link health ใน ROS2
สำหรับ Bool จะเก็บเวลา receipt ฝั่งหุ่นของข้อความนั้น ไม่ refresh ค่า true เก่าเป็น heartbeat ใหม่
สำหรับ Joy จะคำนวณ heartbeat ใหม่จาก sample ที่ผ่านการตรวจเวลา แล้วส่ง false เมื่อ sample หมดอายุ
receiver ที่เปิด driver output ตรวจ gate นี้บนเครื่องหุ่นด้วย และไม่รับ arm ขณะ gate ไม่อนุญาต
deadman เป็นการอนุญาตจากผู้ควบคุม ส่วนการตรวจ footprint/traversability และ LiDAR ยังต้องครบตามหัวข้อก่อนเคลื่อนที่

### เลือกที่ต่อเว็บแคม

- เสียบกับคอม Jazzy: คง `camera.source="local_usb"` ใช้ camera.index เดิม
- เสียบกับคอม ROS1: ตั้ง `camera.source="ros1_compressed"` และ
  `camera.ros1_compressed_topic` เป็น topic `sensor_msgs/CompressedImage` ของ **ภาพ raw RGB** เช่น `/husky_vision_camera/image_raw/compressed`
  gateway ส่งภาพแบบ bounded/latest ผ่าน JPEG/PNG และ publish `/husky_link/image_raw` ให้ detector
  header timestamp/frame และขนาดพิกเซลเดิมถูกเก็บไว้ ไม่ resize หรือเปลี่ยน stamp ให้ดูใหม่
- มีกล้อง ROS2 publisher อยู่แล้ว: ตั้ง `camera.source="external_ros2"` และเปิด pipeline ด้วย `image_topic:=/actual/image_raw`

สำหรับเว็บแคมบน ROS1 ที่ยังไม่มี driver เตรียม launch ไว้ให้ โดยเลือก device/pixel_format ที่กล้องรองรับจริง:

```bash
sudo apt install ros-${ROS_DISTRO}-usb-cam ros-${ROS_DISTRO}-compressed-image-transport
roslaunch husky_vision_bridge webcam.launch video_device:=/dev/video2
```

ค่าเริ่มต้น 640×480, 15 fps, mjpeg, frame=`husky_vision_camera_optical_frame`
ต้องตรงกับ calibration, raw capture mode และ measured mount; PNG/JPEG แบบ RGB เท่านั้น ไม่ใช้ compressedDepth
อย่าเลือกภาพ rectified แล้วใช้ distortion ของ raw camera ซ้ำ
ถ้าหุ่นมี camera TF ของ child เดียวกันอยู่แล้ว ใช้ frame/mount ที่ตรวจตรงกันและตั้ง `camera.publish_camera_tf=false`
เพื่อให้ GroundPath node ไม่ส่ง camera TF ซ้ำ
การเปิด UVC driver/คุณภาพภาพจริงต้องตรวจบน webcam/เครื่องหุ่น ส่วน network codec ทดสอบด้วยภาพสังเคราะห์

`base_to_ground_z_m` เป็นค่าที่วัดจริง เช่น origin ของ base สูงกว่าพื้นจะใช้ค่า Z ติดลบ
launch สร้างเฉพาะ measured base -> ground -> camera; odom -> base ต้องมาจากหุ่นจริง
ถ้า TF ของหุ่นส่ง child ของกล้องเดียวกันอยู่แล้ว ต้องจัดให้มีผู้ส่ง TF child นั้นเพียงรายเดียว
ชุดนี้ใช้พื้นราบและการติดตั้งที่วัดไว้ การเคลื่อนบนพื้นลาด/การเปลี่ยนท่าทางต้องปรับ geometry เพิ่ม

## เปิด connection แบบอ่านข้อมูล

คัดลอก `robot.json` ที่เติมแล้วไว้ที่ `~/husky_ros1_ws/robot.json` บนหุ่นด้วย
ROS1 terminal ที่ source workspace แล้ว:

```bash
roslaunch husky_vision_bridge bridge.launch \
  profile_path:="$HOME/husky_ros1_ws/robot.json"
```

rosbridge bind 127.0.0.1:9090 เริ่มด้วย output ไป `/husky_vision/watchdog_preview`
ยังไม่ส่งให้ driver และไม่ arm receiver ใช้ ROS1 topics เฉพาะ prefix /husky_vision/
service/parameter access ผ่าน rosbridge ถูกปิดไว้ ต้องใช้ operator terminal บนฝั่งหุ่นสำหรับ arm

บนเครื่อง Jazzy เปิดอีก terminal ทำ SSH tunnel ค้างไว้ โดยแทน USER/ROBOT_IP ด้วยบัญชีและ IP จริง:

```bash
ssh -N -L 9090:127.0.0.1:9090 USER@ROBOT_IP
```

Terminal ที่ source Jazzy/venv/workspace แล้ว:

```bash
ros2 launch husky_vision robot_link.launch.py
```

ดู `/husky_link/status`, `/husky_link/odom`, `/husky_link/scan` และ TF
ไม่ต้องตั้ง ROS_MASTER_URI ให้ชี้ข้ามเครื่องสำหรับ gateway นี้
นาฬิกาสองเครื่องต้องตรงกัน ตรวจ source age; ไม่แก้ stamp ให้เป็นเวลาปัจจุบันเพื่อซ่อน network delay
ถ้าเน็ตหลุด client จะปิด enable latch; เชื่อมต่อใหม่ไม่ทำให้กลับมาส่งคำสั่งเอง
ถ้ากล้อง ROS1 ส่งภาพนิ่งแต่ภาพ Jazzy ขาดช่วง ดู [การวัดและอัปเดต camera transport](https://github.com/jorjor140900/husk/blob/work/docs/ros1-camera-latency.md)
มีการแยก watchdog จาก image telemetry, จำกัดภาพล่าสุด และ status แยกช่วงเวลาในการรับ/ถอดภาพ

## เปิด pipeline รวมหลังกรอกค่าหุ่นครบ

หยุด robot_link/launch กล้องเดิมก่อน แล้วรันจาก Jazzy:

```bash
ros2 launch husky_vision robot_pipeline.launch.py \
  profile:="$HOME/husky_vision_ws/robot/robot.json"
```

launch รวม webcam, detector, measured TF/ground projection, controller, LiDAR filter และ link
controller ยัง disabled และ `bridge.forward_commands` เริ่ม false
ไม่มี publisher ที่สร้าง odometry/scan/permission ปลอมให้ผ่านเงื่อนไข
เปิดดูภาพแยกเมื่อจำเป็นด้วย image_view และ compressed transport ตามคู่มือ ROS2 เดิม
launch รวมเริ่ม headless เพื่อเหลือ CPU ให้ inference และ control

```bash
ros2 run husky_vision robot_preflight \
  --profile "$HOME/husky_vision_ws/robot/robot.json"
```

ได้ report ใน `outputs/robot-preflight/` และ PASS/PENDING ตามข้อมูลจริง exit 2 หมายถึงยังมีรายการค้าง
ตรวจอายุ odom/scan/path, สถิติ source age ที่รับจริงและ p95 ของเส้นอ้างอิงเทียบ 500 ms,
ความตรงของนาฬิกา, odometry frames, TF ณ capture time, รูปแบบ scan และ heartbeat ของ gate อิสระ
Empty Path ไม่นับเป็นเส้นใหม่ที่ผ่านเกณฑ์เวลา แม้ header จะเป็นเวลาปัจจุบัน
preflight ไม่ arm, ไม่ส่ง command และไม่ปรับนาฬิกาหรือพารามิเตอร์ให้

## ข้อที่ยังต้องจัดให้ครบก่อนเปิดการเคลื่อนที่

การเชื่อม topic สำเร็จยังต้องผ่านการตรวจภาคสนามเหล่านี้:

สำหรับ cpr-a200-1068 พบ VLP16 driver ที่สร้าง scan จาก ring 8 (+1°), metadata ไม่ตรงกับ
สัญญาของตัวกรอง และมี acquisition ประมาณ 101 ms แม้ LaserScan timing เป็นศูนย์
ดู [ผลตรวจ bag และขอบเขต adapter offline](https://github.com/jorjor140900/husk/blob/work/docs/vlp16-field-review.md)
ยังต้องทำ runtime integration/coverage ก่อนเปิดคำสั่ง ห้ามถือว่า time_increment=0 ยืนยัน deskew แล้ว

- พิกัดกล้อง/ระยะบนพื้นและ footprint ที่วัดจริง
- path/drivable/obstacle map ต้องรองรับ footprint และพื้นที่กวาดตอนเลี้ยว ผ่าน gate อิสระ
  ที่ส่ง Bool heartbeat บน `robot.external_permission_topic`; ค่าเริ่มต้นรับ local gate ที่ bridge มา
  false/ขาด heartbeat จะหยุด และสามารถใช้ gate ROS2 แยกเพิ่มเติมได้โดยเปลี่ยน topic ให้ตรง
  ไม่มี node ที่ตั้ง true ให้เพื่อข้ามขั้นนี้
- ตัวกรอง LiDAR ปัจจุบันรับ scan 360° ที่ instantaneous/deskewed เท่านั้น
  ถ้า inventory/preflight ขึ้น `scan_requires_deskew` หรือ `incomplete_scan` ต้องทำ adapter/deskew/coverage ที่ตรงกับ sensor จริงก่อน
  ห้ามตั้ง time_increment=0 หรือเติมจุดที่ไม่ได้วัดเป็น inf เพื่อให้ผ่าน
- perception ต้องทัน path timeout 0.5 s จาก source stamp; ลด latency จากการวัด
- คำสั่งต้องเข้าช่อง autonomy ของ mux ที่ตรวจแล้ว และฐานหุ่นมี watchdog/E-stop ที่ใช้งานได้

ฟิลด์ `field_measurements` เป็นบันทึกว่าตรวจจริงแล้ว ไม่ใช่ผลที่ self-test ตั้งให้โดยอัตโนมัติ
`ready_for_autonomous_control` จึงยัง false ในรายงานของ gateway/preflight
การทดลองวิ่งจริงต้องตรวจข้อจำกัดของ sensor/geometry/gate เหล่านี้กับหุ่นก่อน

## ลำดับเปิดให้รับคำสั่งเมื่อทดสอบภาคสนามครบแล้ว

1. บันทึกผลตรวจจริงใน profile; ใส่ expected_output_topic ที่ยืนยันแล้ว และตั้ง bridge.forward_commands=true
2. คัดลอก profile ล่าสุดไปหุ่น แล้วเปิด ROS1 launch เพิ่ม `allow_robot_output:=true`
   receiver ยัง disarmed และไม่ยึด mux ด้วยการส่ง zero ค้างเมื่อ inactive
3. เปิด pipeline ใหม่บน Jazzy ตรวจ preflight และทดสอบการหยุด/ทิศแกนโดยผู้ควบคุมอยู่กับ E-stop
4. กด deadman ค้าง/ให้ local gate อนุญาต เปิด link latch, arm receiver ที่ terminal ฝั่ง ROS1, แล้วเปิด controller ตามลำดับ:

```bash
# Jazzy
ros2 service call /husky_link/set_enabled std_srvs/srv/SetBool '{data: true}'
# ROS1 terminal on robot
rosservice call /husky_vision/set_enabled "data: true"
# Jazzy, after the independent permission gate is operational
ros2 service call /husky_control/control/set_enabled std_srvs/srv/SetBool '{data: true}'
```

ความเร็วเริ่มต้นของ field pipeline จำกัด 0.10 m/s, omega 0.30 rad/s
สั่งหยุด controller/link ใช้ service เดิม data:false; receiver ฝั่ง ROS1 ใช้ data:false
เมื่อ command ขาด/เก่าเกิน 0.25 s, ผิด frame/ขอบเขต/replay จะ disarm และเปลี่ยน lease
ส่ง zero ช่วงสั้นแล้วปล่อย mux input ให้ timeout คืนสิทธิ์; operator ต้องเปิดใหม่หลัง fault/reconnect
driver watchdog และ E-stop ของฐานหุ่นยังเป็นส่วนจำเป็นเมื่อ process/เครื่องหุ่นเองมีปัญหา

## การตรวจซอฟต์แวร์และขอบเขตผลทดสอบ

มี unit tests ของ lease/replay/source time/monotonic deadline/limits และการรักษา scan unknowns
มี offline link self-test ด้วย WebSocket + DDS + receiver จำลองใน namespace สุ่ม
ตรวจ build package บน ROS1 Noetic และเชื่อมผ่าน official rosbridge 0.11.17 กับ Jazzy จริงในสภาพทดสอบแยก
ใช้ sensor messages สังเคราะห์และ topic ทดลอง ตรวจ local watchdog, deadman, static TF, unknown scan data และ timestamp เดิม
ตรวจ launch รวมตั้งแต่ Image/model fixture ผ่าน ground path/controller/filter/link โดยเริ่ม observe/disabled
และตรวจ preflight ว่ารายงาน input/TF/permission/latency ที่ค้างตามจริง
ผลซอฟต์แวร์ไม่ได้ยืนยันความแม่นยำ AI, physical braking หรือความพร้อมของ sensor บนหุ่นจริง
ข้อมูลรุ่น ROS1/topic/scan ที่ยังไม่ทราบจะระบุเป็น PENDING แทนการเลือกแทนหุ่นจริง
