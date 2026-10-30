# ใช้ image/AI นำทาง Husky โดยไม่ใช้ joystick

คีย์บอร์ดใช้เริ่ม/หยุดรอบวิ่ง ส่วน image/AI คำนวณเส้นและบังคับเดิน/เลี้ยวเอง
permission topic อยู่ `/husky_operator/permission` นอก namespace ที่ rosbridge อนุญาตให้ส่งคำสั่ง
จึงไม่ให้ช่องส่งคำสั่งภาพ/AI เผยแพร่ permission โดยตรงผ่าน WebSocket
ไม่ต้องเปิด teleop node และไม่ส่ง Twist ตรงข้าม controller ไป `/cmd_vel`
โหมดปกติใช้ LiDAR filter; โหมดทดลองข้าม LiDAR ต้องเลือกอย่างชัดเจนตามขั้น 8
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

### ตอนหุ่นปิด: ดูกล้องพร้อมเส้นเหลือง/ขอบแดงแบบเดิม

โหมดนี้ไม่ใช้ map, TF, odometry หรือ goal และไม่เปิด controller/navigation:

```bash
QT_QPA_PLATFORM=xcb ros2 launch husky_vision vision_field.launch.py \
  profile:="$HOME/husky_vision_ws/robot/robot-nojoy-preview.json" \
  camera_only:=true
```

ในอีก Terminal ที่ source environment และ export Cyclone แล้ว:

```bash
QT_QPA_PLATFORM=xcb ros2 run image_view image_view --ros-args \
  -r image:=/drivable_area/overlay -p image_transport:=compressed \
  -p autosize:=false -p width:=960 -p height:=720
```

ใช้ preview worker 10 FPS ที่ทดลองไว้ หน้าจอระบุ `IMAGE PREVIEW ONLY`
เส้นเหลืองแสดงเมื่อ mask มีเส้นที่ผ่านเงื่อนไข; ขอบแดงเป็นเส้นประมาณจากขอบ mask ที่มองเห็น
หยุด launch และ viewer ชุดนี้ก่อนเปิด field navigation เพื่อไม่ให้กล้อง/publisher ซ้ำ

จอ controller `/husky_nav/vision_overlay` ต่างจาก preview นี้: `waiting_for_goal_guide`
หมายถึงยังรอ goal และเส้น guide ที่ตรงกัน รวมข้อมูลตำแหน่งจริงจากหุ่น
จึงยังไม่มีเส้นควบคุมสีเหลือง ขอบแดงของจอ controller แสดงได้จาก mask แม้ยังไม่มี goal
แต่จะไม่สร้างด้านที่อยู่นอกภาพ/ไม่พบขอบขึ้นมาแทน
ถ้าอยู่ในห้อง ไม่วาง pose/goal บน outdoor map เพื่อฝืนให้เกิดเส้น ใช้ camera_only สำหรับตรวจภาพ

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

**คอม C — enable navigation gate และ link ก่อน:**

```bash
ros2 service call /husky_nav/set_enabled std_srvs/srv/SetBool "{data: true}"
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

- Python suite 343 tests ผ่าน รวม finite session, expiry, stop, state loss, no-auto-rearm
- ทดสอบปุ่มจริงผ่าน PTY โดยแทน ROS services/messages; g, Space, หมดเวลา และ q ผ่าน
- ทดสอบ callback gateway ROS1 จริงร่วมกับ command watchdog โดยใช้ ROS message/service stand-ins
  permission หายขณะคำสั่งยังสดต้องออกศูนย์และ disarm
- exported package build และ field launch `--show-args` ผ่านบน native ROS Jazzy ใน container ที่ไม่มี network
- CLI สร้าง preview profile และ syntax ฝั่งหุ่นผ่าน ยังไม่ได้ทดสอบ ROS1 transport/การเคลื่อนหุ่นจริง

ผลเหล่านี้ไม่ใช่หลักฐานระยะหยุดจริง, VLP16 coverage หรือความแม่นยำของเส้นภาคสนาม


## 8. โหมดทดลองมีคนเฝ้า: ข้ามการตรวจ LiDAR อย่างชัดเจน

ใช้เฉพาะการทดลองระยะสั้นที่ผู้ควบคุมมองเห็นหุ่นและทางตลอด พร้อมใช้ hardware e-stop
โหมดนี้ **ไม่ตรวจสิ่งกีดขวางด้วย LiDAR** และไม่เปลี่ยนผล field measurements ให้เป็น true
mask จาก RGB อาจมองพื้น/สิ่งกีดขวางผิดได้ การมีเส้นเหลืองไม่รับรองว่าหุ่นผ่านได้

จำกัดความเร็ว .03 m/s, เลี้ยว .15 rad/s, ระยะสะสม .5 m, เวลา 20 s และมุมเลี้ยวสะสม .35 rad
ถึงข้อใดก่อนจะหยุด ต้องเริ่มรอบใหม่เอง ระยะเหล่านี้เป็นจุดสั่งหยุดจาก odometry ไม่ใช่ระยะหยุดทางกายภาพที่รับรอง
จำกัดทั้งฝั่ง ROS2 และ gateway บนหุ่น จึงต้องอัปเดตทั้งสองเครื่องตามขั้น 1–2 ก่อนใช้
เวลา gateway เริ่มนับตั้งแต่ arm ส่วน ROS2 เริ่มนับเมื่อมีคำขอความเร็วครั้งแรก
ภาพเก่า, reference หาย, odometry/TF ขาด, permission/คำสั่งขาด หรือ link หลุดยังทำให้หยุด
หลังเริ่มเคลื่อนที่แล้ว fault จะค้าง แม้ข้อมูลกลับมาสดก็ไม่วิ่งต่อเอง

### สร้างและคัดลอกไฟล์ใหม่

หลัง git pull/export/build ตามขั้น 1 บนคอม (หยุด launch เดิมก่อน):

```bash
ros2 run husky_vision prepare_operator_profile \
  --profile "$HOME/husky_vision_ws/robot/robot-nojoy-preview.json" \
  --output "$HOME/husky_vision_ws/robot/robot-nojoy-trial.json" \
  --robot-output "$HOME/husky_vision_ws/robot/ros1-nojoy-trial.json" \
  --supervised-trial
scp -r "$HOME/husky_vision_ws/src/husky_vision/ros1/husky_vision_bridge" \
  administrator@192.168.131.1:~/husky_ws/src/
scp "$HOME/husky_vision_ws/robot/ros1-nojoy-trial.json" \
  administrator@192.168.131.1:~/husky-nojoy-trial.json
```

หากชื่อ output มีอยู่แล้วให้ใช้ไฟล์เดิมที่สร้างด้วยรุ่นนี้หรือเลือกชื่อใหม่ เครื่องมือไม่เขียนทับ
บนหุ่น build package ใหม่ตามขั้น 2; ปิด console ด้วย q แล้วหยุด gateway/field launch เดิมก่อนเปลี่ยนชุด
ทุก terminal ใช้ environment เดิมในขั้น 1–2; ROS2 ต้อง export Cyclone เหมือนกันทั้งหมด
SSH tunnel เดิมเปิดค้างไว้ได้ หากยังไม่มี เปิดบนคอมอีก terminal:

```bash
ssh -N -o ExitOnForwardFailure=yes \
  -L127.0.0.1:9090:127.0.0.1:9090 administrator@192.168.131.1
```

### เปิดรอบทดลอง

**หุ่น A — gateway:**

```bash
roslaunch husky_vision_bridge bridge.launch \
  profile_path:="$HOME/husky-nojoy-trial.json" \
  odom_topic:=/odometry/filtered scan_topic:=/scan base_frame:=base_link \
  allow_robot_output:=true supervised_trial:=true
```

**คอม B — กล้อง/AI/navigation:**

```bash
QT_QPA_PLATFORM=xcb ros2 launch husky_vision vision_field.launch.py \
  profile:="$HOME/husky_vision_ws/robot/robot-nojoy-trial.json" \
  map:="$HOME/husky_vision_ws/maps/outdoor1.yaml" \
  allow_motion:=true supervised_trial:=true
```

ตั้ง initial pose ให้ตรงพื้นที่จริง แล้วเลือก goal ใหม่ตรงทางเดิน
ดู `/husky_nav/vision_overlay` ซึ่งแสดงเส้นเหลืองที่ controller ใช้จริง พร้อมแถบ `SUPERVISED: LiDAR OFF`
หาก goal ไกลเกิน .5 m รอบนี้จะหยุดก่อนถึง goal ตามขีดจำกัดระยะ

**หุ่น B — console เปิดค้างไว้ ยังไม่กด g:**

```bash
rosrun husky_vision_bridge operator_console.py \
  _profile_path:="$HOME/husky-nojoy-trial.json" \
  _allow_robot_output:=true _supervised_trial:=true _session_sec:=20.0
```

**คอม C — enable ทั้งสอง gate หลัง console เริ่มแล้ว:**

```bash
ros2 service call /husky_nav/set_enabled std_srvs/srv/SetBool "{data: true}"
ros2 service call /husky_link/set_enabled std_srvs/srv/SetBool "{data: true}"
```

ทั้งคู่ต้องตอบ success=true จากนั้นกด **g** ใน console หุ่น B เพียงครั้งเดียว
AI/controller ส่งความเร็วตามเส้นเหลือง ส่วน console อนุญาต/หยุดเท่านั้น ไม่ต้องใช้ joystick หรือ teleop
**Space หยุด, q หยุดและออก**; hardware e-stop สำหรับเหตุฉุกเฉิน

สถานะ ROS2 `/husky_control/safety/status` จะระบุ `model: supervised_vision_no_lidar`, `lidar_checked: false`
`scan_input_reason: invalid_scan_metadata` อาจยังแสดงได้ เพราะเก็บสถานะ scan เดิมไว้ แต่ไม่ได้ใช้ตัดสินการเคลื่อนที่ในโหมดนี้
สถานะบนหุ่น `/husky_vision/trial_status` แสดง armed, reason, ระยะ/มุมสะสมและขีดจำกัด
โหมดปกติยังปฏิเสธ scan เดียวกันตามเดิม

### เริ่มรอบใหม่หลังหยุด/fault/ครบขีดจำกัด

กด Space ให้ console เป็น stopped ก่อน แล้ว reset ฝั่งคอม:

```bash
ros2 service call /husky_nav/reset_trial std_srvs/srv/Trigger "{}"
ros2 service call /husky_nav/set_enabled std_srvs/srv/SetBool "{data: true}"
ros2 service call /husky_link/set_enabled std_srvs/srv/SetBool "{data: true}"
```

reset ต้องตอบ success=true; จะถูกปฏิเสธขณะ permission ยังเปิดอยู่
ตรวจตำแหน่ง/ทางและ goal ใหม่ก่อนกด g อีกครั้ง ไม่มีการต่อเวลา/arm อัตโนมัติ
หากสั่งหยุดจาก terminal ให้ใช้วิธีในขั้น 7 ได้ตามเดิม

### ผลทดสอบโหมดนี้

Python suite 360 tests ผ่าน รวม profile+flag ต้องตรงกัน, ไม่แก้ field facts, จำกัดความเร็วโดยรักษาความโค้ง,
งบระยะ/เวลา/มุม และ gateway หุ่นหยุดแม้คำสั่งยังสด
Native ROS Jazzy ใน container ที่ตัด network ใช้ navigation gate และ safety node จริงร่วมกับข้อมูลจำลอง:
โหมด trial ส่งออกไม่เกิน .03 m/s ขณะที่โหมดปกติปฏิเสธ scan เดียวกัน;
ภาพ/odom หายแล้วหยุดค้าง, reset ขณะ permission เปิดถูกปฏิเสธ, ครบระยะ/เวลาแล้วหยุด และไม่มี publisher `/cmd_vel`
ดู fixture `tests/native_supervised_trial_probe.py` (ใช้เฉพาะ container network none)
ยังไม่ได้ทดสอบการวิ่งหรือระยะหยุดจริงบนหุ่น


### ถ้า controller รอ capture TF ทั้งที่ tf2_echo ล่าสุดมี TF

รุ่นหลัง 75bbaa9 แก้กรณีภาพ 15 FPS มาเร็วกว่าข้อมูล TF: เดิม segmentation ใหม่เขียนทับ
ภาพที่กำลังรอทุกครั้ง จึงอาจรอ TF ของภาพล่าสุดตลอดโดยไม่เคยสร้าง reference สำเร็จ
ตอนนี้เก็บภาพรอเพียงหนึ่งภาพ ใช้ timestamp เดิม และ retry จนได้ TF หรือหมดอายุ 500 ms
เส้นที่สร้างสำเร็จและยังสดไม่ถูกล้างเพียงเพราะ TF ของภาพถัดไปยังมาไม่ถึง
ไม่เลื่อน stamp ไม่ใช้ latest TF แทนเวลา capture และไม่เพิ่มงบภาพเก่า
packet ผิด/goal ยกเลิก/ภาพหมดอายุยังตัดเส้นและให้ศูนย์

อัปเดตเฉพาะ ROS2 บนคอมตามขั้น 1 แล้ว restart field launch; ไม่ต้องเปลี่ยน trial profile
หรือ build gateway หุ่นใหม่สำหรับการแก้นี้ ตั้ง pose/goal ใหม่และ enable ทั้งสอง gate ก่อนเริ่มรอบใหม่
ตรวจ status `tracking` และเส้นเหลืองจริงที่ `/husky_nav/vision_overlay`
หากยังรอ status จะมี `capture_tf_error` บอก transform ที่ขาดและช่วงเวลาให้ตรวจต่อ


### ถ้า enable สำเร็จแต่กลับเป็น false ก่อนกด g

ในโหมดปกติ `receiver_state_stale_or_clock_skew` ทำให้ link ล้าง enable latch แม้ gateway ยังไม่ arm
ดังนั้นสถานะกลับ `reason: ok` ภายหลังไม่ได้แปลว่าเปิด command forwarding กลับแล้ว
หากกด g หลัง enable ถูกล้าง gateway อาจจบด้วย `no_first_command`; คำสั่งที่ขาดช่วงภายหลังเป็น `command_timeout`
อย่าเปิด auto-reenable หรือเพิ่ม watchdog เพื่อกลบเหตุนี้

`/husky_link/status` มี `disable_reason` จำเหตุที่ล้าง latch ครั้งแรกจนกว่าจะ enable ใหม่
และ `receiver_timing_ms` แสดง source_age_ms, receipt_age_ms,
source_age_at_receipt_estimate_ms (รวม clock offset และความหน่วงส่ง ไม่ใช่ network delay ล้วน)
receipt_age สูงชี้ว่าสถานะขาดช่วง ส่วน source_age สูงแต่ receipt_age ต่ำชี้ว่าข้อมูลมาถึงก็เก่าแล้วหรือเวลาต่างกัน
การเพิ่ม fields นี้ไม่เปลี่ยนเกณฑ์อายุ ไม่ auto-enable และไม่ส่งคำสั่งใหม่

หยุด operator session แล้วตรวจ route สาย LAN และความหน่วงบนคอม:

```bash
ip route get 192.168.131.1
ping -c 20 -i 0.2 192.168.131.1
timedatectl show -p NTP -p NTPSynchronized
```

บนหุ่นตรวจ `timedatectl show -p NTP -p NTPSynchronized` เช่นกัน
ping ปกติไม่รับรองว่า ROS/rosbridge ไม่ค้าง และ NTP synchronized ทั้งคู่ไม่ได้วัด offset ระหว่างเครื่องโดยตรง
ถ้า ping ปกติแต่ ROS stamps เก่า ให้ตรวจ clock offset, โหลด process/ROS1 gateway/rosbridge และ topic ที่มี payload ใหญ่ต่อ


### โหมดทดลอง: เตรียมรอ g ได้ 20 วินาทีหลัง enable

สำหรับ `supervised_trial:=true` เท่านั้น เมื่อ enable link ขณะ gateway ยัง disarmed
จะเปิดช่วงเตรียมเริ่มไม่เกิน 20 วินาที (`awaiting_receiver_arm: true`)
ช่วงนี้ state เก่าชั่วคราวยังรอข้อมูลใหม่ได้ แต่ไม่ส่งคำสั่งและไม่ให้ motion_permitted ขณะข้อมูลเก่า
จึงไม่สูญเสียการเตรียมเริ่มเพียงเพราะ Wi-Fi สะดุดระหว่างเปลี่ยน terminal ไปกด g

เมื่อเห็น gateway armed ครั้งแรก ช่วงเตรียมสิ้นสุดทันที
ตั้งแต่นั้นข้อมูลเก่า/คำสั่งขาด/การ disarm จะปิดค้างตามเดิม ไม่เปิดใหม่เมื่อข้อมูลกลับมา
ไม่มีการเพิ่ม receiver timeout .3 s หรือ command watchdog .25 s
connection หลุด, คำสั่ง disable, lease เปลี่ยนขณะยัง disarmed หรือครบ 20 วินาทีจะยกเลิกช่วงเตรียม
หากหมดช่วงเตรียม ให้ console อยู่ STOP แล้ว enable ใหม่เอง ไม่กด g เพื่อฝืนให้เริ่ม
โหมดปกติไม่เปลี่ยนพฤติกรรม

อัปเดตเฉพาะ ROS2 ตามขั้น 1 และ restart field launch เดิมพร้อม supervised_trial:=true
profile และ ROS1 gateway เดิมใช้ได้ ไม่ต้องเปลี่ยน flags การตรวจภาคสนาม
ทดสอบร่วมกับ CommandWatchdog ฝั่งหุ่นจริงใน Python: สะดุดก่อน arm ไม่ส่งคำสั่งเก่า,
arm แล้วรับคำสั่งศูนย์แรก ตามด้วย .03 m/s, ขาดข้อมูลหลัง arm ต้องหยุด และ arm รอบใหม่ไม่ทำให้ link เปิดเอง
การทดสอบนี้ไม่ใช่การขยับหุ่นจริง


### Filter ค้าง trial_navigation_gate_closed ทั้งที่ link active

รุ่นถัดจาก be34b2f เริ่ม motion budget ฝั่ง ROS2 เมื่อความเร็วที่ไม่เป็นศูนย์ออกจาก filter ครั้งแรก
หลังผ่าน clear hold .5 s แล้ว ช่วงก่อนหน้านั้น gate ที่ยังไม่พร้อมยังทำให้ความเร็วศูนย์และเริ่มรอใหม่
แต่ไม่ latch ว่าเสียระหว่างวิ่ง ทั้ง watchdog และงบ 20 s ฝั่งหุ่นนับตั้งแต่ arm ตามเดิม
เมื่อมีความเร็วออกแล้ว ข้อมูล/permission/gate หายยังหยุดค้างและต้อง reset ขณะ STOP

status filter มี `navigation_gate_reason` (ปัจจุบัน) และ `trial_fault_context.navigation_gate.reason`
(เหตุครั้งแรกที่ทำให้หยุดหลังเริ่มส่งความเร็ว เก็บไว้จน reset) จึงตรวจหลัง session expired ได้
ไม่ต้องกด g อีกรอบเพียงเพื่ออ่านเหตุเก่า

ตัวจับทั้งรอบเปิดก่อน enable/g บนคอมใน ROS2 environment ใช้จาก checkout ได้โดยไม่ build:

```bash
python3 ~/husk-live/tools/trace_trial_session.py --duration 60
```

เป็น subscriber อย่างเดียว ไม่มี publisher ความเร็ว/permission และไม่มีการ enable/reset ให้อัตโนมัติ
ในอีก terminal reset/enable ตามขั้น 8 แล้วกด g บนหุ่นหนึ่งรอบ
ตัวจับจะจบหลังเห็น disarm 1 วินาที หรือครบ 60 วินาที แล้วพิมพ์ `REPORT:` พร้อมไฟล์ .json
ส่งไฟล์รายงานนั้นได้แม้ session จบแล้ว รายงานมีเหตุ gate ขณะ armed,
ครั้งแรกที่ gate ปิดหลังเคยผ่าน และค่าสูงสุดของ filtered command ส่วน .jsonl เป็นข้อความทุก sample
ไม่มีภาพกล้อง และไม่ได้รับรองการขยับ/ระยะหยุดจริง


### เส้นเหลือง tracking แต่ gate บอก vision_invalid

รายงาน trial-20261030T052148027582Z พบ max_abs_filtered_linear_mps=0,
ขณะ armed มี vision_invalid 84 samples และผ่าน request เพียง 5 samples
ตอน gate ปิด link ยัง enabled/armed และ source age ราว 72 ms;
controller เส้นเหลืองรายงาน tracking แต่ gate เดิมอ่าน `/husky_nav/vision_status` จากตัวฉาย grid แยกต่างหาก
ตัว grid รายงาน invalid ระหว่างรอ capture TF ของภาพถัดไป จึงบล็อกเส้นที่ controller ใช้ได้

ใน steering_mode=vision ตอนนี้ remap vision status ของ navigation gate ไป
`/husky_nav/vision_control_status` ซึ่งมี valid=true เฉพาะเมื่อ controller demand เป็น tracking
source_stamp_ns ยังเป็นเวลา capture ที่สร้าง reference จริง ไม่ใช่เวลาส่ง status 20 Hz
ภาพเกิน 500 ms, ไม่มี reference/goal/TF/odom ที่ใช้ได้, mask ถูกบล็อก หรือ cancel จะให้ valid=false
nav2 mode ยังคงตรวจ grid status เดิม ส่วน LiDAR filter/permission/watchdog และขีดจำกัด trial ไม่เปลี่ยน
status gate เพิ่ม vision_reason เพื่อบอกเหตุจาก input ที่ gate ใช้ และ fault context เก็บเหตุนี้ได้

อัปเดตเฉพาะคอมตามขั้น 1 แล้ว restart field launch เดิมพร้อม trial profile
ไม่ต้องเปลี่ยน profile, gateway หุ่น หรือยืด timeout
ทดสอบ Python 369 tests ผ่าน และ fixture `tests/native_reference_gate_probe.py` ใน native ROS2 ที่ตัด network:
controller+gate+filter จริงส่ง filtered command .03 m/s ได้แม้ legacy grid invalid ตลอดและ TF ช้า 120 ms
ภาพหมดอายุและ mask ถูกบล็อกยังให้ศูนย์และ latch ไม่กลับไปเคลื่อนเองหลังข้อมูลกลับมา
ไม่มี publisher `/cmd_vel` ใน fixture และไม่ได้ขยับหุ่นจริง


## 9. วิ่งถึง goal ใกล้ด้วยความเร็ว 0.05 m/s (มีคนเฝ้า)

ใช้ profile ใหม่ `--supervised-goal` สำหรับ goal ใกล้ประมาณ 1–2 m ที่กำลังทดลอง
หยุดที่ goal ตาม controller (ระยะคลาดเคลื่อนยอมรับ .15 m) ไม่ถูกตัดที่ .5 m/20 s ของชุดเดิม
มีเพดานสำรองระยะสะสม 3 m, 120 s, มุมเลี้ยวสะสม .35 rad และเลี้ยวสูงสุด .15 rad/s ทั้งคอมและหุ่น
ยังข้ามการตรวจ LiDAR; คนเฝ้าต้องเห็นเส้นทางและพร้อมใช้ hardware e-stop
ภาพเก่า/mask ใช้ไม่ได้/odom/TF/permission/คำสั่งขาดยังหยุด ไม่มี auto-restart หลัง fault
เพดานนี้ไม่ใช่การรับรองระยะหยุดทางกายภาพหรือว่าผ่านสิ่งกีดขวางได้

profile trial เดิมยังเป็น .03 m/s, .5 m/20 s ตามเดิม ไม่มีการแอบเปลี่ยนค่าของไฟล์เก่า
ต้องอัปเดต **ทั้งคอมและ gateway บนหุ่น** แล้วใช้ profile ใหม่ที่คู่กัน

### อัปเดตและสร้างไฟล์บนคอม

กด Space แล้ว q ใน console หุ่น หยุด field launch และ gateway เดิมก่อนเปลี่ยน package
SSH tunnel ไป `10.82.12.199` เปิดค้างไว้ได้ ใช้ IP Wi-Fi ของหุ่นที่ยืนยันล่าสุด

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
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run husky_vision prepare_operator_profile \
  --profile "$HOME/husky_vision_ws/robot/robot-nojoy-trial.json" \
  --output "$HOME/husky_vision_ws/robot/robot-nojoy-goal.json" \
  --robot-output "$HOME/husky_vision_ws/robot/ros1-nojoy-goal.json" \
  --supervised-goal

scp -r "$HOME/husky_vision_ws/src/husky_vision/ros1/husky_vision_bridge" \
  administrator@10.82.12.199:~/husky_ws/src/
scp "$HOME/husky_vision_ws/robot/ros1-nojoy-goal.json" \
  administrator@10.82.12.199:~/husky-nojoy-goal.json
```

ถ้า output มีอยู่แล้ว ใช้ไฟล์ที่สร้างด้วยรุ่นนี้หรือเลือกชื่อใหม่ เครื่องมือไม่เขียนทับ
ไม่แก้ผล field measurements เป็น true

### Build และเปิด gateway บนหุ่น

```bash
ssh administrator@10.82.12.199
source /opt/ros/noetic/setup.bash
cd ~/husky_ws
catkin_make --pkg husky_vision_bridge
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
unset ROS_HOSTNAME
export ROS_IP=10.82.12.199

roslaunch husky_vision_bridge bridge.launch \
  profile_path:="$HOME/husky-nojoy-goal.json" \
  odom_topic:=/odometry/filtered scan_topic:=/scan base_frame:=base_link \
  allow_robot_output:=true supervised_trial:=true
```

### เปิด field และ console

บนคอม terminal ที่ source environment/Cyclone ตามขั้น 1 แล้ว:

```bash
QT_QPA_PLATFORM=xcb ros2 launch husky_vision vision_field.launch.py \
  profile:="$HOME/husky_vision_ws/robot/robot-nojoy-goal.json" \
  map:="$HOME/husky_vision_ws/maps/outdoor1.yaml" \
  allow_motion:=true supervised_trial:=true
```

ภาพ `/husky_nav/vision_overlay` ต้องมีแถบ `SUPERVISED GOAL` และ .05 m/s, backup 3m/120s
ตั้ง pose และ goal ใหม่ตรงตำแหน่งจริง รอเส้นเหลือง

บนหุ่น terminal อีกอันที่ source Noetic/devel และ ROS_MASTER_URI/ROS_IP เหมือน gateway:

```bash
rosrun husky_vision_bridge operator_console.py \
  _profile_path:="$HOME/husky-nojoy-goal.json" \
  _allow_robot_output:=true _supervised_trial:=true _session_sec:=120.0
```

console ต้องแสดง `one 120s session` เปิดค้างไว้ที่ STOP ก่อน
อย่าใช้คำสั่งเก่าที่มี `_session_sec:=20.0` เพราะจะยังหมดเวลา 20 s ตามที่ระบุเอง

บนคอมเปิดตัวเก็บรอบใน terminal แยกก่อนเริ่ม:

```bash
python3 ~/husk-live/tools/trace_trial_session.py --duration 90
```

จาก terminal คอมอีกอัน:

```bash
ros2 service call /husky_nav/reset_trial std_srvs/srv/Trigger "{}"
ros2 service call /husky_nav/set_enabled std_srvs/srv/SetBool "{data: true}"
ros2 service call /husky_link/set_enabled std_srvs/srv/SetBool "{data: true}"
```

ทุกคำสั่งต้อง success=true แล้วกด g ครั้งเดียวใน console ภายในช่วงเตรียม 20 s
ช่วงเตรียมนี้เป็นเวลารอเริ่ม ยังไม่ใช่อายุ session วิ่ง 120 s
ถึง goal controller จะให้ศูนย์และ latch goal_reached ไม่เริ่ม goal ใหม่เอง
console อาจยังแสดง active ขณะที่ความเร็วเป็นศูนย์ ให้กด Space/q เพื่อจบรอบ
หากชนเพดานสำรองหรือเกิด fault ต้อง STOP/reset/enable ก่อนเริ่มรอบใหม่
สถานะ gateway ควรแสดง max_linear_mps=.05 และ safety/status มี trial_limits 3m/120s
ถ้ายังเห็น .03 หรือ .5m/20s แปลว่ายังใช้ package/profile ฝั่งใดฝั่งหนึ่งเก่า อย่าปรับไฟล์แบบสุ่ม

### ผลทดสอบ

Python 374 tests ผ่าน รวมเก็บ trial เดิมไม่เปลี่ยน, ปฏิเสธ preset ที่แก้เอง,
profile สองฝั่งตรงกัน, callback gateway รับ .05 m/s และหยุดที่ backup limit
syntax shared core/scripts ผ่าน Python 3.8
Native ROS2 ใน container ที่ตัด network ใช้ controller+gate+filter จริงและภาพ/odom จำลอง
วิ่งจำลอง 1.257 m ใน 25.9 s ที่ความเร็วสูงสุด .05 m/s แล้วหยุดในรัศมี goal .144 m
จึงผ่านขีดจำกัดเก่า .5m/20s และหยุดด้วย goal จริงใน fixture
TF หน่วง 120 ms, legacy grid invalid ตลอด, ไม่มี publisher /cmd_vel และไม่มีการเชื่อมต่อหุ่นจริง
ดู `tests/native_goal_session_probe.py`; ไม่ใช่ผลทดสอบการเคลื่อน/หยุดบนหุ่นจริง
