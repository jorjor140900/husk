# ใช้ image/AI นำทาง Husky โดยไม่ใช้ joystick

คีย์บอร์ดใช้เริ่ม/หยุดรอบวิ่ง ส่วน image/AI คำนวณเส้นและบังคับเดิน/เลี้ยวเอง
permission topic อยู่ `/husky_operator/permission` นอก namespace ที่ rosbridge อนุญาตให้ส่งคำสั่ง
จึงไม่ให้ช่องส่งคำสั่งภาพ/AI เผยแพร่ permission โดยตรงผ่าน WebSocket
ไม่ต้องเปิด teleop node และไม่ส่ง Twist ตรงข้าม controller/LiDAR filter ไป `/cmd_vel`
RViz ใช้เลือกตำแหน่งและ goal ส่วนเส้นควบคุมจริงอยู่ที่ `/husky_nav/vision_reference`
และภาพ `/husky_nav/vision_overlay` เส้น preview เดิม `/drivable_area/overlay`
เป็นภาพทดลอง จึงไม่ใช้เป็นจออ้างอิงการขับจริง

**สถานะ:** ซอฟต์แวร์เตรียมโหมดไม่ใช้จอยแล้ว ยังไม่ได้ยืนยันการวิ่ง/ระยะหยุดจริง
VLP16 ที่พบก่อนหน้านี้ส่ง scan สำหรับ mapping; runtime safety/coverage ยังต้องตรวจ
`navigation_preflight` ไม่รับรอง obstacle safety และค่า `field_measurements` ไม่ถูกตั้ง true โดยเครื่องมือ

## พฤติกรรมเริ่ม/หยุด

- Console บนหุ่นเริ่ม STOPPED ส่ง permission=false ไม่มีการเริ่มวิ่งจากการเปิดโปรแกรม
- `g` ภาษาอังกฤษ ไม่ต้องกด Enter: อนุญาตหนึ่ง session 30 วินาที แล้วขอ arm gateway หลัง permission เริ่มส่งแล้ว
- Space หรือปุ่มอื่นนอกจาก `g`: หยุด, `q`: หยุดและออก, Ctrl+C/ปิด SSH: หยุดส่ง heartbeat
- เมื่อครบเวลา session จะหยุด กด g ซ้ำขณะกำลังทำงานไม่ต่อเวลาให้อัตโนมัติ
  ปรับ `_session_sec:=60.0` ได้ สูงสุด 120 วินาทีต่อรอบ
- ถ้า gateway หลุด/disarm/state เก่า session จะหยุด ไม่กลับมาวิ่งเองเมื่อเชื่อมต่อใหม่
- ต้อง enable ROS2 link ใหม่ก่อนกด g ใหม่หลังการหยุดทุกครั้ง
- Console ไม่มี publisher ความเร็ว มีเฉพาะ Bool permission ที่ gateway ตรวจอายุรับไม่เกิน .3 s
  gateway ยังตรวจ lease, stamp, ลำดับคำสั่ง และ timeout คำสั่ง .25 s ตามเดิม
- ปุ่มหยุดของคอนโซลเป็น software stop ไม่ใช่ hardware e-stop; ใช้ e-stop หุ่นสำหรับเหตุฉุกเฉิน

## 1. เตรียมที่บ้านในคอม Jazzy

ปิด launch กล้อง/AI/navigation เดิมก่อน update:

```bash
cd ~/husk-live
git pull --ff-only origin work
python3 tools/create_ros2_workspace.py \
  --workspace "$HOME/husky_vision_ws" --update-code
cd ~/husky_vision_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
python /usr/bin/colcon build --symlink-install --packages-select husky_vision
source install/local_setup.bash
```

ทุก Terminal ROS2 ใช้ชุดนี้ รวม viewer, preflight และ service commands:

```bash
source /opt/ros/jazzy/setup.bash
source ~/husky_vision_ws/.venv/bin/activate
source ~/husky_vision_ws/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

หากยังไม่ได้ติดตั้ง Cyclone: `sudo apt install ros-jazzy-rmw-cyclonedds-cpp`
CUDA/RTX 3050 ต้องยังใช้งานได้ (`nvidia-smi`) เลือก kernel ที่ตรวจแล้วว่าเปิดได้
ไม่ต้องติดตั้ง driver ใหม่เพื่อใช้โหมดนี้

สร้าง **ไฟล์ใหม่** สำหรับทดสอบไม่เคลื่อนที่ โดยไม่ทับ robot.json เดิม:

```bash
ros2 run husky_vision prepare_operator_profile \
  --profile "$HOME/husky_vision_ws/robot/robot.json" \
  --output "$HOME/husky_vision_ws/robot/robot-nojoy-preview.json" \
  --robot-output "$HOME/husky_vision_ws/robot/ros1-nojoy-preview.json"
```

ได้ joy_topic=null, joy_deadman_button=null, ROS1 permission `/husky_operator/permission`
และ mux input `/cmd_vel` ซึ่งตรวจพบจาก `/twist_mux/topics` ของหุ่นนี้แล้ว
geometry, mount, margins และ camera settings เดิมถูกเก็บไว้
ผลตรวจ permission ของจอยเดิมใช้แทน console ใหม่ไม่ได้ จึงตั้งผลตรวจ permission
ในไฟล์ใหม่เป็น false เมื่อเปลี่ยน source; ถ้าเปลี่ยน mux input ก็ล้างผลตรวจ mux ในไฟล์ใหม่เช่นกัน
ไฟล์ต้นฉบับไม่ถูกแก้ และเครื่องมือไม่ตั้งผลตรวจใดเป็น true
หาก output มีอยู่แล้วให้ใช้ไฟล์นั้นหรือเลือกชื่อใหม่ เครื่องมือไม่เขียนทับ

## 2. หน้างาน: เชื่อมต่อและอัปเดต package บนหุ่น (ครั้งแรกของรุ่นนี้)

เปิดหุ่นตามปกติ ใช้สาย LAN และตั้ง IP คอมตามเครือข่ายที่ใช้งานจริง
ตัวอย่างหุ่นเดิมอยู่ `192.168.131.1` ให้ตรวจ `ssh` ว่ายังเข้าได้ก่อน
จากคอม Jazzy:

```bash
scp -r "$HOME/husky_vision_ws/src/husky_vision/ros1/husky_vision_bridge" \
  administrator@192.168.131.1:~/husky_ws/src/
scp "$HOME/husky_vision_ws/robot/ros1-nojoy-preview.json" \
  administrator@192.168.131.1:~/husky-nojoy-preview.json
ssh -t administrator@192.168.131.1
```

บนหุ่น:

```bash
source /opt/ros/noetic/setup.bash
cd ~/husky_ws
catkin_make --pkg husky_vision_bridge
source devel/setup.bash
```

เป็นการอัปเดต package งานนี้ ไม่เปลี่ยน bringup หลัก/teleop/mux ของโรงงาน
ทุก SSH Terminal ฝั่งหุ่นใช้:

```bash
source /opt/ros/noetic/setup.bash
source ~/husky_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
unset ROS_HOSTNAME
export ROS_IP=192.168.131.1
```

## 3. เปิดรับข้อมูลและลอง goal โดยยังไม่ให้เคลื่อนที่

**Terminal หุ่น A — gateway preview:**

```bash
roslaunch husky_vision_bridge bridge.launch \
  profile_path:="$HOME/husky-nojoy-preview.json" \
  odom_topic:=/odometry/filtered scan_topic:=/scan \
  base_frame:=base_link allow_robot_output:=false
```

**Terminal คอม A — tunnel เปิดค้างไว้:**

```bash
ssh -N -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:9090:127.0.0.1:9090 administrator@192.168.131.1
```

**Terminal คอม B — กล้อง + CUDA AI + map/AMCL + controller + RViz:**
ต้องปิด webcam/AI/navigation ชุดแยกเดิมก่อน เพื่อไม่เปิด publisher ซ้ำ

```bash
QT_QPA_PLATFORM=xcb ros2 launch husky_vision vision_field.launch.py \
  profile:="$HOME/husky_vision_ws/robot/robot-nojoy-preview.json" \
  map:="$HOME/husky_vision_ws/maps/outdoor1.yaml" \
  allow_motion:=false
```

launcher ใช้ stable camera ID เดิม, raw size/FPS จาก profile และ camera frame จาก mount
ใช้ CUDA model 256×160, CPU threads=2, Cyclone DDS และ `steering_mode=vision`
สร้างเส้นสำหรับ controller โดยตรง ไม่เปิด worker เส้น pixel preview เก่าซ้ำ
ถ้า camera ID ต่าง ใช้ `camera_device:=/dev/v4l/by-id/ชื่อกล้องจริง-video-index0`

ใน Terminal คอม C หลังกล้องเปิดแล้ว ตั้ง focus ให้ตรง calibration ทีละคำสั่ง:

```bash
v4l2-ctl -d /dev/v4l/by-id/usb-Generic_Web_Camera_200901010001-video-index0 --set-ctrl=focus_automatic_continuous=0
v4l2-ctl -d /dev/v4l/by-id/usb-Generic_Web_Camera_200901010001-video-index0 --set-ctrl=focus_absolute=120
v4l2-ctl -d /dev/v4l/by-id/usb-Generic_Web_Camera_200901010001-video-index0 --set-ctrl=zoom_absolute=0
```

RViz: Fixed Frame=map; ใช้ 2D Pose Estimate วางตำแหน่งจริงและลูกศรชี้หน้าหุ่น
ถ้า AMCL active แต่ map TF ไม่เกิดขณะหุ่นหยุด:

```bash
ros2 service call /request_nomotion_update std_srvs/srv/Empty "{}"
```

เลือก goal ใหม่ประมาณ 1 เมตรตรงหน้ากลางทางเดิน โดยดูระยะบน map ไม่ใช่กะจากหน้าจอ

**Terminal คอม D — ภาพเส้นที่ controller ใช้จริง:**

```bash
QT_QPA_PLATFORM=xcb ros2 run image_view image_view --ros-args \
  -r image:=/husky_nav/vision_overlay -p image_transport:=compressed \
  -p autosize:=false -p width:=960 -p height:=720
```

**Terminal คอม C — ตรวจข้อมูลสดและสาเหตุที่ยังหยุด:**

```bash
ros2 run husky_vision navigation_preflight \
  --profile "$HOME/husky_vision_ws/robot/robot-nojoy-preview.json" \
  --map "$HOME/husky_vision_ws/maps/outdoor1.yaml" --live --duration 10
ros2 topic echo /husky_nav/vision_control_status --once --full-length
ros2 topic echo /husky_control/safety/status --once --full-length
```

`motion_not_permitted` เป็นปกติในรอบนี้ ส่วน `scan_input_reason` ต้องแยกตรวจแม้ยังไม่อนุญาตเคลื่อน
ถ้าเป็น `invalid_scan_metadata`, `invalid_scan_ranges`, `scan_requires_deskew`, ไม่มี scan ฯลฯ
ยังเปิด autonomous movement ไม่ได้ ต้องแก้แหล่งข้อมูล/การฉาย LiDAR จริงก่อน ไม่ปิด filter
`scan_input_reason=ok` แปลว่า metadata/ranges ผ่านตัว parser เท่านั้น ไม่รับรอง coverage/ระยะหยุด
โดยเฉพาะ `/husky_nav/mapping_scan` แก้ metadata เพื่อ mapping ไม่ใช่ safety scan ทดแทน

## 4. ทดสอบคีย์บอร์ดโดยไม่เคลื่อนที่ก่อน

**Terminal หุ่น B (SSH แบบ `ssh -t`) — เปิด console:**

```bash
rosrun husky_vision_bridge operator_console.py \
  _profile_path:="$HOME/husky-nojoy-preview.json" \
  _allow_robot_output:=false _session_sec:=30.0
```

รอ state แล้วกด g สังเกต priming → waiting_arm → active
ลอง Space, g ใหม่, ปิด console และปล่อยครบเวลา ทุกครั้งต้องกลับ permission=false และ gateway disarmed
เปิด console เพียงหนึ่งตัว ไม่ใช้ `rostopic pub ... true` แบบค้างแทน console
โหมดนี้ output อยู่ `/husky_vision/watchdog_preview` และฝั่ง ROS2 ไม่ forward จึงไม่มีการขับจริง
watchdog อาจ disarm ใน 1 วินาทีเพราะไม่มีคำสั่งแรกใน observe mode; นี่ไม่ใช่ความผิดพลาดของ joystick
สำหรับทดสอบ local permission/expiry ต่อเนื่อง ใช้ unit tests; อย่าส่งคำสั่งปลอมเพื่อฝืนให้ค้าง armed บนหุ่น

## 5. เงื่อนไขก่อนสร้าง motion profile (ทำครั้งแรก/เมื่อเปลี่ยนฮาร์ดแวร์)

ยังต้องทำและบันทึกผลจริงของ:

1. ขนาดหุ่น/ส่วนยื่น/mount และ scan-to-map alignment; corridor ต้องพอสำหรับทั้งตัวหุ่น
2. ช่อง `/cmd_vel` เป็น external input mux จริง, priority, base/mux timeout และการหยุดเมื่อสัญญาณหาย
3. console permission false/หมดอายุ/ปิด SSH ต้องทำให้ gateway disarm และหุ่นหยุดจริง
4. LiDAR obstacle coverage และระยะหยุดจริงของ final filter รวม blind area ใกล้หุ่น

สามรายการใน `robot-nojoy-preview.json.field_measurements` ใช้บันทึกผลที่ทดสอบแล้วเท่านั้น
การตรวจ permission ต้องเป็น console/source ใหม่นี้ ไม่ยกผลตรวจ joystick เดิมมาใช้
ค่า true ไม่ใช่ผลที่เครื่องมืออนุมานจาก software tests; ข้อ LiDAR ยังต้องตรวจแยกแม้ flags ครบ
หากยังทำไม่ได้ให้จบที่ preview ไม่ใช้คำสั่งส่งตรงเข้า `/cmd_vel` เพื่อข้ามสิ่งที่ขาด

เมื่อบันทึกผลครบและ safety input/coverage ใช้งานได้แล้ว สร้าง motion files ใหม่บนคอม:

```bash
ros2 run husky_vision prepare_operator_profile \
  --profile "$HOME/husky_vision_ws/robot/robot-nojoy-preview.json" \
  --output "$HOME/husky_vision_ws/robot/robot-nojoy-motion.json" \
  --robot-output "$HOME/husky_vision_ws/robot/ros1-nojoy-motion.json" \
  --motion
scp "$HOME/husky_vision_ws/robot/ros1-nojoy-motion.json" \
  administrator@192.168.131.1:~/husky-nojoy-motion.json
```

ถ้าแจ้ง checks pending ต้องกลับไปทดสอบและบันทึกผล ไม่ใช่เปลี่ยน flags เพื่อให้ launch ผ่าน

## 6. รอบวิ่งจริง: เปิด output แต่ยังไม่ start จนกด g

หยุด navigation preview และ console/gateway preview ด้วย Ctrl+C ก่อน
กล้อง/AI/RViz ของ field launch ปิดพร้อม navigation; tunnel ค้างไว้ได้

**หุ่น A:**

```bash
roslaunch husky_vision_bridge bridge.launch \
  profile_path:="$HOME/husky-nojoy-motion.json" \
  odom_topic:=/odometry/filtered scan_topic:=/scan \
  base_frame:=base_link allow_robot_output:=true
```

**คอม B:**

```bash
QT_QPA_PLATFORM=xcb ros2 launch husky_vision vision_field.launch.py \
  profile:="$HOME/husky_vision_ws/robot/robot-nojoy-motion.json" \
  map:="$HOME/husky_vision_ws/maps/outdoor1.yaml" \
  allow_motion:=true
```

ตั้ง focus เดิมหลังเปิดกล้อง ตั้ง initial pose ใหม่ และเลือก goal ใหม่ใกล้ ๆ
ตรวจเส้น `/husky_nav/vision_overlay` และตำแหน่งจริงก่อนเริ่ม
ค่าเริ่มต้นความเร็ว controller .05 m/s เลี้ยวสูงสุด .2 rad/s

**หุ่น B — console เปิดค้างไว้ แต่ยังไม่กด g:**

```bash
rosrun husky_vision_bridge operator_console.py \
  _profile_path:="$HOME/husky-nojoy-motion.json" \
  _allow_robot_output:=true _session_sec:=30.0
```

**คอม C — enable link ก่อน:**

```bash
ros2 service call /husky_link/set_enabled std_srvs/srv/SetBool "{data: true}"
```

ต้องตอบ success=true จากนั้นกลับ console หุ่น B กด **g**
controller จะส่งคำสั่งตามเส้น image/AI เมื่อ localization/reference/permission/LiDAR ผ่านพร้อมกัน
ถ้าไม่ขยับอ่าน `vision_control_status`, `safety/status`, `husky_link/status` ไม่ publish ความเร็วข้าม gate

ถึง goal แล้ว controller ให้ศูนย์; หากครบ 30 วินาทีก่อนถึง goal console จะหยุด
ไม่เริ่ม goal ใหม่เอง การเริ่มอีกครั้งต้องตรวจสภาพหน้างาน enable link ใหม่ แล้วกด g ใหม่

## 7. หยุด

ปกติ: กด **Space** ใน console หุ่น B; `q` เพื่อหยุดแล้วออก
สำรองใน Terminal หุ่นอีกอัน:

```bash
rosservice call /husky_vision/set_enabled "data: false"
```

ฝั่งคอม:

```bash
ros2 service call /husky_link/set_enabled std_srvs/srv/SetBool "{data: false}"
ros2 service call /husky_nav/cancel_vision_goal std_srvs/srv/Trigger "{}"
```

ถ้า LAN ขาด ให้ใช้ stop/e-stop บนหุ่น ไม่พึ่งคำสั่งที่ต้องวิ่งผ่าน network ที่ขาด
การปิดคอนโซล/ขาด heartbeat จะทำให้ local permission หมดอายุ แต่ระยะหยุดทางกายภาพต้องวัดจริง
หลัง reconnect ทั้งสองฝั่งไม่ auto-arm; ต้องเริ่ม session ใหม่

## ผลทดสอบซอฟต์แวร์ของการเปลี่ยนนี้

- Python suite 340 tests ผ่าน รวม finite session, expiry, stop, state loss, no-auto-rearm
- ทดสอบปุ่มจริงผ่าน PTY โดยแทน ROS services/messages; g, Space, หมดเวลา และ q ผ่าน
- ทดสอบ callback gateway ROS1 จริงร่วมกับ command watchdog โดยใช้ ROS message/service stand-ins
  permission หายขณะคำสั่งยังสดต้องออกศูนย์และ disarm
- exported package build และ field launch `--show-args` ผ่านบน native ROS Jazzy ใน container ที่ไม่มี network
- CLI สร้าง preview profile และ syntax ฝั่งหุ่นผ่าน ยังไม่ได้ทดสอบ ROS1 transport/การเคลื่อนหุ่นจริง

ผลเหล่านี้ไม่ใช่หลักฐานระยะหยุดจริง, VLP16 coverage หรือความแม่นยำของเส้นภาคสนาม
