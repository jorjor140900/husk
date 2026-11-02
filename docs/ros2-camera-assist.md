# ช่วยเลี้ยวจากกล้องขณะกดเดินหน้าด้วย teleop

โหมดนี้ไม่มี goal, waypoint, map, Nav2, AMCL หรือ RViz ไม่มีการรับ LaserScan ในตัวเลือกทางและตัวกรองคำสั่ง ใช้ USB camera → SegFormer → mask พื้นที่เขียว → เส้นช่วยเลี้ยว → คำสั่งที่ผ่าน permission/watchdog เดิมของหุ่น

คนต้องส่งคำสั่งเดินหน้าค้างจาก teleop จึงมีความเร็วออก ระบบเลือกมุมเลี้ยวแทนค่า angular จาก teleop ไม่รองรับถอยหรือหมุนอยู่กับที่ในโหมดนี้ ปุ่มหยุดส่งศูนย์ได้ทันที การเปิด launch หรือ enable อย่างเดียวไม่ทำให้วิ่ง

กล้องเห็นเฉพาะบางพื้นที่และโมเดลอาจจำแนกผิด โหมดนี้ไม่รับประกันการหลบสิ่งกีดขวาง/ขอบทางทุกกรณี พื้นที่บอดใกล้หุ่นใช้ข้อสมมติการคุมทดลองเดิม ต้องมีผู้คุมและปุ่มหยุดจริง Odometry ยังต้องสดเพื่อชดเชย reference ระหว่างกล้องกับหุ่นและตรวจงบการเคลื่อนที่ ไม่ใช้ map localization

## อัปเดตและเปิด

หยุด launch navigation/vision_field และ teleop เดิมก่อน ไม่เปิดพร้อมกันกับ camera_assist

```bash
cd ~/husk-live
git pull --ff-only origin experiment/local-ai-lidar
python3 tools/create_ros2_workspace.py --workspace "$HOME/husky_vision_ws" \
  --update-code --update-detector
cd ~/husky_vision_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python /usr/bin/colcon build --symlink-install --packages-select husky_vision
source install/local_setup.bash
```

ใช้ SSH tunnel, ROS1 bridge และ supervised junction profile คู่เดิมบนหุ่นต่อได้ ไม่ต้องเปิด SLAM หรือ localization บนคอม ตัวอย่างใช้ profile ล่าสุดที่ตั้ง .15 m/s, .15 rad/s, 100 m/900 s, มุมสะสม 2π (ไม่ต่อเวลา session อัตโนมัติ)

```bash
ros2 launch husky_vision camera_assist.launch.py \
  profile:="$HOME/husky_vision_ws/robot/robot-nojoy-junction.json" \
  allow_motion:=true
```

ไม่ใส่ `map`, `waypoint_mode` หรือ goal หากยังไม่เชื่อมหุ่น ใช้ `allow_motion:=false` จะเปิดเฉพาะกล้อง, AI และตัวสร้างภาพ ไม่เปิด bridge/permission/filter และไม่ต้องมี TF/odometry เส้น preview คำนวณจากภาพปัจจุบันและไม่ถูกนำไปส่งคำสั่งมอเตอร์ ภาพเผยแพร่ที่ `/husky_assist/overlay/compressed` ใช้ image viewer เดิมเลือก topic นี้ได้:

```bash
ros2 run rqt_image_view rqt_image_view /husky_assist/overlay
```

## Teleop ต้องส่งเข้าตัวช่วยเลี้ยว ไม่ใช่ส่งเข้าหุ่นตรง

เปิดอีก terminal ที่ source ROS2/workspace และ RMW เหมือนกัน ใช้ teleop ที่มากับโหมดนี้:

```bash
ros2 run husky_vision camera_assist_teleop
```

กด **i / w / ลูกศรขึ้นค้าง** = ขอเดินหน้า ระบบปรับเลี้ยวให้ตาม mask; **k / Space / ปุ่มอื่น** = ศูนย์ทันที; Ctrl+C = ปิด teleop และส่งศูนย์ (หากโปรแกรม/การสื่อสารหาย input จะหมดอายุภายใน 0.2 s)

terminal ไม่มี key-up event จึงใช้ timeout หลัง key-repeat สุดท้าย การปล่อยปุ่มหยุดภายในประมาณ 0.6 s + รอบ timer ไม่ใช่หยุดทันทีตอนยกนิ้ว ไม่ใช้คำสั่งแบบ latch ค้างตลอด ถ้า keyboard repeat ของเครื่องเริ่มช้ากว่า 0.6 s อาจหยุดช่วงเริ่มกด สามารถลด key_timeout ได้เมื่อ repeat ของเครื่องรองรับ ตัวอย่าง `--ros-args -p key_timeout:=0.3` (ช่วงที่ยอมรับ 0.1..0.6 s)

หากจะใช้ teleop ตัวอื่น ให้ publish **Twist** ไป `/husky_assist/teleop_cmd_vel` เท่านั้น ต้องมี stop-on-release/timeout และส่ง heartbeat ระหว่างกด ไม่ใช้ teleop แบบ latch เดินหน้าตลอดหลังปล่อยปุ่ม และไม่ส่งตรง `/cmd_vel` หรือ filtered command

## Enable และเริ่ม session

หยุด permission ที่คอนโซลบนหุ่นก่อน reset:

```bash
ros2 service call /husky_assist/reset_trial std_srvs/srv/Trigger '{}'
ros2 service call /husky_assist/set_enabled std_srvs/srv/SetBool '{data: true}'
ros2 service call /husky_link/set_enabled std_srvs/srv/SetBool '{data: true}'
```

จากนั้นกด g ที่ operator console บนหุ่นเหมือนเดิมเพื่ออนุญาต session แล้วกด i ค้างใน terminal teleop ไม่ต้องปักโกลหรือสั่ง waypoints/start ใช้ Space ที่ operator console หยุด/ตัด permission ได้เหมือนเดิม

## เส้นและ mask

ตัววางเส้นค้นกราฟในพื้นที่เขียวที่ฉายลงพื้นด้วย calibration, กันความกว้างหุ่นและ margin, เลือกแนวต่อเนื่องที่ยาวและอยู่กลางทาง แล้วทำเส้นให้เรียบพร้อมตรวจพื้นที่ตลอดความกว้างซ้ำ ไม่มีการ fit เส้นแดงเพื่อวางเส้น

ตัวกรอง mask ใช้ optical flow จับคู่ภาพก่อนเฉลี่ย probability และ hysteresis: เริ่มเป็นทางที่ .62, พื้นที่ที่เคยเป็นทางและยังจับคู่ภาพได้ใช้ threshold คงสถานะ .54 โดย current probability ต้องไม่ต่ำกว่า .50 พื้นที่ vegetation/explicit veto และ current negative ที่ชัดเจนยังชนะทันที ไม่เติมหญ้า/non-drivable ให้เขียว

เฟรมช้าไม่รีเซ็ตประวัติ segmentation หากประวัติจากกล้องเดิมยังอายุไม่เกิน 1 s แต่ไม่ได้ทำให้เฟรมเก่ากลายเป็นเฟรมสด เมื่อกล้องขาด/เปลี่ยนฉาก/ข้ามเวลาจะล้าง history ตามเดิม

reference ล่าสุดอยู่ใน odom และชดเชยตามการเคลื่อนหุ่น ใช้ได้สูงสุด 2 s จาก timestamp เดิม หลัง .5 s ลดความเร็วไม่เกิน .05 m/s; กลับใช้ความเร็วจาก teleop เมื่อภาพใหม่รองรับ ถ้าหมดอายุหรือ mask ใหม่ห้ามผ่านให้พักศูนย์ และกลับมาเดินได้เมื่อมีเส้นใหม่ใน session เดิม โดยต้องยังกดเดินหน้าและ permission ยังอยู่ ไม่ reset งบหรือ re-arm เอง

ตรวจสถานะ:

```bash
ros2 topic echo /husky_assist/status --once --full-length
ros2 topic echo /drivable_area/status --once --full-length
ros2 topic echo /husky_control/safety/status --once --full-length
```

เส้นเหลืองในภาพคือ geometry ของ controller บน source capture เดียวกัน ภาพที่ค้างไม่ได้แปลว่าคำสั่งยังออก ดู `reason`, `reference_age_ms`, `linear_x` และสถานะ safety ประกอบด้วย

## เปิดทุก terminal ตั้งแต่ต้น (คอม Jazzy / หุ่น Noetic)

ตัวอย่างใช้ IP ล่าสุด `10.82.13.0` ถ้าเครือข่ายเปลี่ยนให้แทน IP ทุกคำสั่ง โปรไฟล์ junction คู่เดิมต้องมีอยู่แล้ว ไม่ต้องสร้างหรือเขียนทับโปรไฟล์ใหม่ Base driver และ `/odometry/filtered` ฝั่งหุ่นต้องทำงานตาม bringup ปกติ หยุด field launch, mapping, RViz, teleop และ console session เก่าก่อนเริ่ม

### เตรียมครั้งเดียวหลัง pull/build บนคอม

อัปเดต ROS1 package ที่ export แล้วและใช้โปรไฟล์คู่เดิม:

```bash
scp -r "$HOME/husky_vision_ws/src/husky_vision/ros1/husky_vision_bridge" \
  administrator@10.82.13.0:~/husky_ws/src/
scp "$HOME/husky_vision_ws/robot/ros1-nojoy-junction.json" \
  administrator@10.82.13.0:~/husky-nojoy-junction.json
ssh administrator@10.82.13.0
source /opt/ros/noetic/setup.bash
cd ~/husky_ws
catkin_make --pkg husky_vision_bridge
exit
```

### Terminal 1 — คอม: tunnel เปิดค้าง

```bash
ssh -N -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:9090:127.0.0.1:9090 administrator@10.82.13.0
```

ไม่มี output และ prompt ไม่กลับเป็นปกติของ tunnel ถ้า port ถูกใช้ ให้ใช้ tunnel เดิมที่ทำงานอยู่ ไม่เปิดซ้ำ

### Terminal 2 — คอม SSH เข้าหุ่น: bridge เปิดค้าง

```bash
ssh administrator@10.82.13.0
source /opt/ros/noetic/setup.bash
source ~/husky_ws/devel/setup.bash
roslaunch husky_vision_bridge bridge.launch \
  profile_path:="$HOME/husky-nojoy-junction.json" \
  odom_topic:=/odometry/filtered base_frame:=base_link \
  allow_robot_output:=true supervised_trial:=true
```

ไม่ต้องเปิด LiDAR เพื่อให้ camera assist วางเส้นได้ ถ้า profile เดิมยังมี scan_topic ตัว bridge อาจส่ง scan telemetry แต่โหมดช่วยเลี้ยวไม่ใช้ข้อมูลนั้น ตรวจว่าไม่ใช่ `allow_robot_output:=false` มิฉะนั้น gateway จะไม่ส่งออกมอเตอร์

### Terminal 3 — คอม SSH เข้าหุ่น: operator console เปิดค้าง

```bash
ssh -t administrator@10.82.13.0
source /opt/ros/noetic/setup.bash
source ~/husky_ws/devel/setup.bash
rosrun husky_vision_bridge operator_console.py \
  _profile_path:="$HOME/husky-nojoy-junction.json" \
  _allow_robot_output:=true _supervised_trial:=true _session_sec:=900.0
```

คงสถานะ STOPPED ไว้ก่อน ยังไม่กด g; Space ที่ terminal นี้ใช้ตัด permission

### Terminal 4 — คอม: camera assist เปิดค้าง

```bash
source /opt/ros/jazzy/setup.bash
source ~/husky_vision_ws/.venv/bin/activate
source ~/husky_vision_ws/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
QT_QPA_PLATFORM=xcb ros2 launch husky_vision camera_assist.launch.py \
  profile:="$HOME/husky_vision_ws/robot/robot-nojoy-junction.json" \
  allow_motion:=true
```

ถ้ากล้องไม่ใช่ stable device เดิม เพิ่ม `camera_device:=/dev/videoN` ของกล้องจริง ไม่ใส่ map หรือ goal

### Terminal 5 — คอม: ดูภาพ (เลือกเปิด)

```bash
source /opt/ros/jazzy/setup.bash
source ~/husky_vision_ws/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
QT_QPA_PLATFORM=xcb ros2 run rqt_image_view rqt_image_view
```

เลือก `/husky_assist/overlay/compressed` ในรายการ topic ภาพเขียวและเส้นเหลืองจะแสดงได้ก่อนกดเดินหน้า

### Terminal 6 — คอม: reset/enable และเช็กสถานะ

```bash
source /opt/ros/jazzy/setup.bash
source ~/husky_vision_ws/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 service call /husky_assist/reset_trial std_srvs/srv/Trigger '{}'
ros2 service call /husky_assist/set_enabled std_srvs/srv/SetBool '{data: true}'
ros2 service call /husky_link/set_enabled std_srvs/srv/SetBool '{data: true}'
```

แต่ละคำสั่งต้องสำเร็จ จากนั้นกลับ terminal 3 กด **g ภาษาอังกฤษ ไม่ต้อง Enter** เพื่ออนุญาต session แล้วจึงใช้ teleop ถ้าข้อความ `receiver_state_stale_or_clock_skew` กลับมา ต้องแก้นาฬิกา/การเชื่อมต่อก่อน ไม่ใช่ปัญหา mask

### Terminal 7 — คอม: teleop เปิดค้างและใช้กดเดินหน้า

```bash
source /opt/ros/jazzy/setup.bash
source ~/husky_vision_ws/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run husky_vision camera_assist_teleop
```

โฟกัส terminal นี้แล้วกด i/w/Up ค้าง เมื่อทางโค้งระบบปรับ angular ให้เอง ใช้ k/Space หยุดทันที ปล่อยปุ่มหมดอายุภายในประมาณ .6 s; ไม่มีการวิ่งหาโกลเองหลังปล่อย

เมื่อจะจบ ให้กด k/Space ที่ terminal 7 แล้ว Space ที่ terminal 3 จากนั้น Ctrl+C ที่ launch/bridge/tunnel ได้ ถ้าต้อง reset fault ใหม่ ต้อง STOP ที่ terminal 3 ก่อน แล้วทำ reset/enable ตาม terminal 6 อีกครั้ง

## ทดสอบกล้องบนคอมโดยไม่เชื่อมหุ่น

```bash
ros2 launch husky_vision camera_assist.launch.py \
  profile:="$HOME/husky_vision_ws/robot/robot-nojoy-junction.json" \
  allow_motion:=false
```

เปิด rqt_image_view แล้วเลือก `/husky_assist/overlay/compressed` ได้โดยไม่ต้อง tunnel หรือเปิดหุ่น แม้เลือก allow_motion:=true แต่ไม่มีหุ่น ภาพยังแสดงเป็น preview ได้ ส่วน reference ควบคุมยังต้องรอ TF/odometry จริง ไม่มีการสร้าง TF สมมติแทนหุ่น
