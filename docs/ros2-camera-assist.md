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


## เส้นเหลืองกับคำสั่งจริง

ใน allow_motion=true เส้นเหลืองแสดงเฉพาะ reference ควบคุมที่ยังใช้ได้ ยกเลิกการวาด HELD DISPLAY ONLY เมื่อเส้นใช้ไม่ได้ แม้มีเส้นก็ยังต้องมีคำสั่งเดินหน้าสดและ permission ก่อนหุ่นเดิน โหมด allow_motion=false ยังคงแสดง preview สำหรับดูภาพโดยไม่เชื่อมหุ่น

status แยก perception_reason ออกจาก reason ของคำสั่ง พร้อม teleop_age_ms และ teleop_requested_linear_x เพื่อไม่ให้สถานะปล่อยปุ่มบดบังปัญหา mask จุดเริ่มที่ถูกปฏิเสธมี start_veto_ground_xy (เมตร), start_total_samples และ reserved_width_m เพื่อหาว่า mask ปฏิเสธตรงไหน ไม่เดาว่าทุกจุดเป็น noise แล้วขับผ่าน

## จุดเริ่มเส้นเมื่อกล้องเยื้องจากกลางหุ่น

ปรับกราฟให้ตรวจความกว้างด้านข้างแทน circular inflation ที่เคยทำให้พื้นที่ unseen ด้านหลังจุดเริ่มปิดทาง แม้แถบเขียวด้านหน้ากว้างพอ การตรวจเส้นสุดท้ายยังตรวจทั้งความกว้างตามแนวตั้งฉากของเส้นกับ mask ปัจจุบัน หากมองเห็นแกนกลางก่อนเห็นครบความกว้าง สามารถขยับจุดเริ่มไปข้างหน้าได้ไม่เกิน .30 m เฉพาะช่วงที่ไม่มี known non-drivable และต้องพบ ingress ที่เขียวครบความกว้าง ไม่ใช้วิธีนี้ข้ามจุดที่ภาพระบุว่าห้ามผ่าน หากยังปฏิเสธจุดเริ่ม status จะแสดง start_veto_samples/start_unknown_samples หรือ start_lateral_width_rejected เพื่อแยกสาเหตุ


## เริ่มแนววิ่งเยื้องแกนหุ่นภายในพื้นที่เขียว

เมื่อขอบขวาของแถบที่ Y=0 ไม่ผ่าน เช่นจุดที่ Y=-.41 ถึง -.335 m ระบบค้น ingress เยื้องซ้าย/ขวาครั้งละ .05 m ได้ไม่เกิน .20 m โดยเลือกใกล้แกนที่สุดที่ผ่านการตรวจครบความกว้าง ทุก candidate ต้องไม่มี known veto ในแถบ และ ingress ที่เลือกต้องเป็น drivable ทั้งแถบ ไม่เปลี่ยน mask หรือค่าโมเดล จุดเริ่มที่เลือกอยู่ใน reference ที่ Pure Pursuit ใช้จริง โดยแสดง reference_start_lateral_m ใน status เส้นน้ำเงินยังเป็นแกนหุ่น Y=0 ตามเดิม

waiting_for_capture_tf แยกจากการสร้างเส้นไม่ได้ status มี capture_tf_error ระบุกรอบพิกัดหรือเวลาที่หาไม่ได้ ไม่เปลี่ยนไปใช้ TF สมมติในโหมดส่งคำสั่ง เมื่อหุ่นปิดให้ใช้ allow_motion=false เพื่อดู preview


## ลดการหายของเส้นเมื่อเส้นทางแรกไม่ผ่าน

ตัววางเส้นลองปลายทางที่แยกกันได้ถึง 8 จุด แทนการคืนเส้นว่างทันทีเมื่อเส้นทางแรกตรวจความกว้างไม่ผ่าน หากยังหาแนวโค้งไม่ได้ จะลองแนวตรงในแถบเริ่มที่ผ่านการตรวจ โดยลดระยะมองล่วงหน้าจนถึงขั้นต่ำที่กำหนด ทุกแนวยังต้องมีพื้นที่ drivable ครบความกว้างตาม mask ปัจจุบัน ไม่เติมสีเขียวทับ non-drivable และไม่ใช้เส้นแดงเป็นเงื่อนไข

ทั้ง preview และ live ลดการสลับแนวด้วยการใช้แนวเดิมเมื่อ mask ใหม่ตรวจผ่านครบความกว้าง และทางใหม่ไม่ได้เพิ่มระยะทางเกิน .25 m; live แปลงแนวเดิมด้วย TF ณ เวลา capture ก่อนตรวจเสมอ เมื่อแนวเดิมเหลือน้อยกว่า .5 m จะเลือกแนวใหม่หากมี เส้นที่ตรวจผ่านเฟรมใหม่จึงเป็น reference สด ไม่ใช่การต่ออายุคำสั่งเก่าโดยไม่มีหลักฐาน ส่วนกรณีข้อมูลหายยังใช้ hold เดิมแบบจำกัดเวลา

การแก้ชุดนี้ตรวจทานโค้ดเท่านั้น ไม่ได้รันทดสอบตามคำขอผู้ใช้งาน ต้องตรวจอัตราประมวลผลและพฤติกรรมกับภาพจริงในการทดลองถัดไป


## แนวกลางพื้นที่เขียวแยกจากแกนหุ่น

เส้นน้ำเงินยังเป็นแกนหุ่น Y=0 ตาม calibration กล้อง ส่วนเส้นเหลืองเริ่มจากกลางแถบเขียวที่กว้างพอ ไม่จำกัดให้เริ่มเยื้องจากแกนเพียง .20 m และไม่ให้คะแนนดึงปลายทางกลับหา Y=0 อีกต่อไป ตัววางเส้นลองแนวกึ่งกลางช่วงเขียวต่อเนื่องในแต่ละระยะด้านหน้าก่อน แล้วใช้กราฟที่ให้คะแนนใกล้กึ่งกลางพื้นที่เป็นทางสำรอง ทุกแนวยังตรวจความกว้างเต็มก่อนใช้ควบคุม ไม่เชื่อมช่วงเขียวข้าม non-drivable

การใช้แนวเดิมซ้ำต้องอยู่ใกล้แนวใหม่ภายใน .10 m ตลอดช่วงที่ซ้อนกัน เพื่อไม่ตรึงแนวเก่าใกล้แกนหุ่น ขณะกดเดินหน้า Pure Pursuit คำนวณการเลี้ยวจากตำแหน่งหุ่นเข้าหาแนวเหลือง ไม่ย้ายเส้นเหลืองให้เข้าหาหุ่น ขีดจำกัดระยะห่างจากเส้นของ controller ยังคงเดิม หากไกลเกินไปยังหยุดได้

กลางพื้นที่ในที่นี้คือกลางพื้นที่ที่กล้องสังเกตได้ภายในระยะค้นหา ไม่ใช่กลางถนนทั้งหมดที่อยู่นอกภาพ; เมื่อกล้องเห็นขอบไม่ครบหรือมีหลายแขนง แนวกลางจึงอาจเปลี่ยนได้ ไม่ได้เปลี่ยนน้ำหนัก SegFormer และไม่ได้รันทดสอบในชุดแก้นี้ตามคำขอผู้ใช้


## ส่งคำสั่งต่อระหว่างค้นหาเส้นเฟรมใหม่

งานค้นหากราฟและ smoothing ย้ายไป worker เดียว มีคิวเก็บเฉพาะ capture ล่าสุด เพื่อไม่ขวาง callback รับปุ่ม/odometry และ timer ส่งคำสั่ง 20 Hz ผลลัพธ์กลับเข้าตัวควบคุมผ่าน timer บน ROS executor เท่านั้น และต้องผ่าน freshness เดิม .5 s ไม่มีการยืด timestamp เพื่อรับผลที่ช้าเกินไป status เพิ่ม planner_busy เพื่อดูว่ากำลังคำนวณอยู่หรือไม่

ระหว่างรอใช้ reference ล่าสุดใน odom คำนวณ pursuit ใหม่ตามตำแหน่งหุ่น ไม่ replay Twist เก่า หาก mask ใหม่ปฏิเสธเฉพาะปลายเส้น สามารถใช้ส่วนต้นที่ต่อเนื่องและตรวจครบความกว้างแล้วได้ เมื่อยาวอย่างน้อย .5 m โดยตัดก่อนจุดที่ไม่ผ่านและตรวจ normal ของปลายเส้นซ้ำ ไม่ข้ามส่วนที่ถูกปฏิเสธไปต่อส่วนเขียวด้านหลัง

การ hold ขณะยังไม่มีหลักฐานใหม่ยังจำกัด 2 s จาก capture เดิม และลดความเร็วไม่เกิน .05 m/s หลัง .5 s ไม่ใช่การค้างคำสั่งแบบไม่สิ้นสุด ปล่อยปุ่ม/permission หาย/odometry เก่า/เส้นหมดอายุยังหยุดตามเดิม การแก้ครั้งนี้ไม่ได้รันทดสอบตามคำขอผู้ใช้


## ลายพื้นและความเร็วเดินหน้า 0.18 m/s

camera_assist ใช้ config/camera-assist-floor.json ซึ่งต่อยอด temporal filter เดิม เพิ่มการเชื่อมช่อง threshold ขนาดไม่เกิน 128 พิกเซลและรัศมีความหนาไม่เกิน 2 พิกเซล ที่ถูกล้อมด้วยพื้นที่ drivable เท่านั้น ทุกพิกเซลในช่องต้องมี current และ filtered probability อย่างน้อย .50 และไม่ติด vegetation/explicit veto ไม่เติมพื้นที่ที่โมเดลให้คะแนนต่ำกว่า .50 ไม่ใช่การเทรนน้ำหนักใหม่ จึงยังต้องตรวจภาพจริงว่ารอยเส้นในโจทย์อยู่ในเงื่อนไขนี้หรือไม่ ดู enclosed_uncertain_pixels ใน /drivable_area/status เพื่อแยกผลของตัวเชื่อมจากผลของโมเดล

โปรไฟล์ camera assist ใหม่เพิ่มความเร็วเดินหน้าจาก .15 เป็น .18 m/s (20%) โดยคง angular limit .15 rad/s และขีดจำกัด session เดิม การเลี้ยว/hold reference ยังลดความเร็วตามเดิม โปรไฟล์ junction เดิมยังใช้ .15 m/s; ต้องอัปเดตทั้ง ROS2 และ ROS1 แล้วเลือกโปรไฟล์ใหม่จึงได้ความเร็วใหม่

หยุดหุ่นและปิด launch/console/teleop ก่อนอัปเดต บนคอม:

```bash
cd ~/husk-live
git pull --ff-only origin experiment/local-ai-lidar
python3 tools/create_ros2_workspace.py --workspace "$HOME/husky_vision_ws" --update-code --update-detector
cd ~/husky_vision_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
python /usr/bin/colcon build --symlink-install --packages-select husky_vision
source install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run husky_vision prepare_operator_profile   --profile "$HOME/husky_vision_ws/robot/robot-nojoy-junction.json"   --output "$HOME/husky_vision_ws/robot/robot-camera-assist-018.json"   --robot-output "$HOME/husky_vision_ws/robot/ros1-camera-assist-018.json"   --supervised-extended-goal --extended-session-sec 900 --turning-session --camera-assist
scp -r "$HOME/husky_vision_ws/src/husky_vision/ros1/husky_vision_bridge" administrator@10.82.13.0:~/husky_ws/src/
scp "$HOME/husky_vision_ws/robot/ros1-camera-assist-018.json" administrator@10.82.13.0:~/husky-camera-assist-018.json
```

คำสั่งสร้างโปรไฟล์ทำครั้งเดียว ไม่เขียนทับไฟล์เดิม flag extended-goal เป็นชื่อ preset session เดิม ไม่เปิดการวิ่งหา goal ในโหมด camera_assist

บนหุ่น:

```bash
source /opt/ros/noetic/setup.bash
cd ~/husky_ws
catkin_make --pkg husky_vision_bridge
source devel/setup.bash
roslaunch husky_vision_bridge bridge.launch   profile_path:="$HOME/husky-camera-assist-018.json"   odom_topic:=/odometry/filtered base_frame:=base_link   allow_robot_output:=true supervised_trial:=true
```

อีก terminal บนหุ่น:

```bash
source /opt/ros/noetic/setup.bash
source ~/husky_ws/devel/setup.bash
rosrun husky_vision_bridge operator_console.py   _profile_path:="$HOME/husky-camera-assist-018.json"   _allow_robot_output:=true _supervised_trial:=true _session_sec:=900.0
```

คง STOPPED ก่อน reset/enable บนคอม เปิด tunnel คู่เดิม แล้ว launch:

```bash
ros2 launch husky_vision camera_assist.launch.py   profile:="$HOME/husky_vision_ws/robot/robot-camera-assist-018.json" allow_motion:=true
```

reset/enable, กด g และ teleop ตามขั้นตอนด้านบน ใช้ camera_assist_teleop ที่ build ใหม่ ค่าเริ่มต้น .18 m/s; การปล่อยปุ่ม/Space และ watchdog ยังทำงานเหมือนเดิม ไม่มีการรันทดสอบการแก้ชุดนี้ตามคำขอผู้ใช้


## เส้นหายให้พัก session ไม่ล็อก fault

รายการ vision pause ครอบคลุม near_axis_not_observable และ too_far_from_path เพิ่มจากกรณีเส้นหมดอายุ/หาเส้นไม่ได้ ในโหมด camera_steering_assist ยังรวม packet หรือ geometry ของภาพที่ใช้ไม่ได้ และ gate heartbeat ขาดช่วงขณะที่ request, permission และ odometry ยังสด ทุกกรณีส่งความเร็วศูนย์เมื่อ reference ใช้ไม่ได้ แต่ไม่หยุด TrialBudget ไม่ reset เวลา/ระยะ/มุม และไม่ส่งคำสั่ง re-arm เมื่อข้อมูลใหม่ใช้ได้และยังกดเดินหน้า สามารถกลับมาเดินใน session เดิมได้หลัง clear hold เดิม

status เพิ่ม trial_active, trial_fault, trial_elapsed_sec, trial_remaining_sec และ vision_paused เพื่อแยกการพักจาก fault ที่ล็อก session การกดหยุด/permission หาย, request watchdog, odometry, transport และเพดานเวลา/ระยะ/มุมยังทำงานเดิม จึงไม่ได้รับประกันว่าจะ armed ครบ 900 s ทุกสถานการณ์ หาก /husky_link/status ระบุ receiver_disarmed_or_lease_changed ต้องดูเหตุผลจาก gateway เพิ่มเติม ไม่ auto re-arm ข้าม fault นั้น

หลังอัปเดต หากมี fault ค้างจากเวอร์ชันเก่า ให้ STOP แล้ว reset/enable เริ่ม session ใหม่ครั้งเดียวตามขั้นตอนเดิม การแก้ชุดนี้ตรวจทานโค้ด ไม่รันทดสอบตามคำขอผู้ใช้


## Gateway command_timeout ระหว่างรอคำสั่ง

หาก /husky_link/status แสดง disable_reason=receiver_disarmed_or_lease_changed:command_timeout หมายถึง gateway เคยขาดคำสั่งเกิน .25 s แม้ภาพจะกลับมาแล้วก็ไม่ใช่ vision pause บนคอม ส่วน receiver.reason เป็นสถานะล่าสุดและอาจเปลี่ยนเป็น local_permission_lost ภายหลัง

camera_assist เปิด idle_zero_heartbeat ที่ ROS2 bridge: transport thread ส่ง zero command ใหม่เมื่อยังไม่มีคำสั่งจาก safety หรือคำสั่งอายุเกิน .10 s โดยเผื่อเวลาส่งก่อน watchdog .25 s ฝั่งหุ่น timestamp ใหม่ใช้กับความเร็วศูนย์เท่านั้น ไม่เปลี่ยน timestamp ของคำสั่งเดินเก่า หลังส่งศูนย์ต้องรอคำสั่งสดที่ stamp ใหม่กว่าศูนย์นั้นก่อนส่งความเร็วอีกครั้ง status เพิ่ม idle_zero_heartbeat_enabled และ command_wait_reason

heartbeat ศูนย์ช่วยคง session เดิมขณะ upstream พัก ไม่ reset budget ไม่ re-arm และไม่ทำงานเมื่อ bridge disabled หรือ receiver ไม่ armed/ข้อมูล receiver ไม่สด Gateway ยังคงหยุดเมื่อ network ขาด, permission หาย หรือครบขีดจำกัดเดิม คำสั่งผิดรูปแบบหรือ telemetry decode ผิดยังไม่ถูกซ่อนเป็น heartbeat การแก้ครั้งนี้อยู่ฝั่ง ROS2 ไม่ต้องเปลี่ยน watchdog บนหุ่น ไม่ได้รันทดสอบตามคำขอผู้ใช้


## ไม่เลือกเส้นสั้นหน้าสิ่งกีดขวางก่อนทางอ้อม

แก้การรับ centreline เส้นแรกทันทีแม้หยุดสั้นหน้าสิ่งกีดขวาง: จัดลำดับผู้สมัครตามระยะต่อด้านหน้าเป็นช่วง .20 m ก่อนใช้ความใกล้กลางพื้นที่/clearance เป็นตัวเลือกในช่วงเดียวกัน กราฟสำรองผู้สมัครด้านซ้าย กลาง ขวา อย่างละไม่เกิน 3 จุด เพื่อไม่ให้ทุกตัวเลือกกระจุกที่ด้านเดียว ในพื้นที่ตรงที่ระยะต่อเท่ากันยังเลือก centreline ก่อน และยังตรวจ mask เต็มความกว้างตาม normal/curvature ทุกเส้น ไม่เติมช่อง non-drivable ให้เป็นทาง

ตัวคง reference เดิมไม่เก็บเส้นสั้นไว้เมื่อแนวใหม่ต่อด้านหน้าได้มากกว่า .15 m เพิ่ม reference_selection และ area_reachable_forward_span_m ใน assist status เพื่อแยกกรณีกราฟไปต่อได้แต่ตรวจเส้นไม่ผ่านจากกรณีไม่มีทางที่เห็นกว้างพอ

การแก้ไม่เพิ่มขอบเขตภาพ ไม่รับรองว่าจะอ้อมได้เมื่อช่องแคบกว่าหุ่น/อยู่นอกกล้อง และไม่ได้เปลี่ยนโมเดลหรือปิดการหยุด เส้นที่ตรวจไม่ผ่านยังถูกปฏิเสธ ไม่ได้รันทดสอบตามคำขอผู้ใช้ คลิปที่แนบกับคำขอนี้เป็นโปรแกรม AprilTag/Cabbage จึงยังไม่ได้ยืนยันอาการจากคลิป Husky จริง


## จูนหุ่นเข้าหาแนวเหลืองเร็วขึ้น

camera assist ลด pursuit lookahead จาก .50 เป็น .35 m และเพิ่ม feedback มุมไปหา target (.60 rad/s ต่อ rad) กับมุมแนวเส้นจาก chord ยาวประมาณ .30 m (.35 rad/s ต่อ rad) ในพิกัดหุ่น เมื่อยังเยื้องแนวจะลดความเร็วเดินหน้า โดยไม่ลด heading correction ตามความเร็วซ้ำอีกครั้ง คงเพดาน angular ของโปรไฟล์เดิม .15 rad/s และจำกัด curvature ของคำสั่งไม่เกิน 4 /m ไม่หมุนอยู่กับที่หรือถอย การถือ reference เก่ายังใช้คำสั่ง pursuit แบบเดิมที่ความเร็วไม่เกิน .05 m/s ไม่เร่ง heading feedback ด้วยภาพเก่า

status มี steering_mode=heading_and_pursuit, alignment_heading_error_rad และ alignment_target_bearing_rad ส่วนเส้นน้ำเงินยังแสดงแกนหุ่นจริงและเส้นเหลืองยังเป็นกลางทาง ไม่ย้ายภาพเส้นแทนการควบคุม รุ่นนี้อัปเดต ROS2 ฝั่งคอมและเปิด launch ใหม่ได้ ไม่ต้องเพิ่ม angular limit ของ gateway บนหุ่น ไม่ได้รันทดสอบตามคำขอผู้ใช้ จึงยังต้องตรวจว่าการแก้ทิศไวขึ้นโดยไม่แกว่งกับหุ่นจริง


## เพิ่มเดินตรงอีก 30%: 0.234 m/s

เพิ่ม preset CAMERA_ASSIST_FAST_LIMITS ที่ .234 m/s (=.18×1.30) คง angular .15 rad/s, acceleration และการลดความเร็วเมื่อไม่ตรงแนว/กำลังเลี้ยว/ถือ reference เก่า โปรไฟล์ .18 เดิมยังใช้ได้และไม่เพิ่มความเร็วเอง ไม่ได้รันทดสอบตามคำขอผู้ใช้

หลัง pull/export/build ฝั่งคอมตามขั้นตอนอัปเดตด้านบน ให้สร้างโปรไฟล์ใหม่หนึ่งครั้ง:

```bash
ros2 run husky_vision prepare_operator_profile   --profile "$HOME/husky_vision_ws/robot/robot-camera-assist-018.json"   --output "$HOME/husky_vision_ws/robot/robot-camera-assist-0234.json"   --robot-output "$HOME/husky_vision_ws/robot/ros1-camera-assist-0234.json"   --supervised-extended-goal --extended-session-sec 900 --turning-session   --camera-assist --camera-assist-speed 0.234
scp -r "$HOME/husky_vision_ws/src/husky_vision/ros1/husky_vision_bridge" administrator@10.82.13.0:~/husky_ws/src/
scp "$HOME/husky_vision_ws/robot/ros1-camera-assist-0234.json" administrator@10.82.13.0:~/husky-camera-assist-0234.json
```

บนหุ่นหยุด bridge/console เดิมและ build package ใหม่ (source Noetic, cd ~/husky_ws, catkin_make --pkg husky_vision_bridge, source devel/setup.bash) แล้วเปิด bridge กับ operator console โดยเปลี่ยน profile_path และ _profile_path ทั้งสองจุดเป็น $HOME/husky-camera-assist-0234.json บนคอมเปิด:

```bash
ros2 launch husky_vision camera_assist.launch.py   profile:="$HOME/husky_vision_ws/robot/robot-camera-assist-0234.json" allow_motion:=true
```

reset/enable และกด g ตามเดิม ต้องใช้ teleop ที่อัปเดตใหม่ซึ่งขอ .234 m/s และตรวจ trial_limits.max_linear_mps กับ receiver.max_linear_mps ว่าเป็น .234 ทั้งคู่ ความเร็วจริงอาจต่ำกว่านี้ระหว่างเร่งหรือแก้ทิศ
