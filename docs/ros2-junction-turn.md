# เลี้ยวแยกตาม goal ด้วย image_only

รุ่นนี้เพิ่ม junction arc ที่ปลายเส้นมี heading เลี้ยวจริง ไม่บังคับกลับขนานกับหน้าหุ่น
รองรับ goal ด้านข้างได้ถึง 110° ใน image_only (ชุดอื่นยังคง 65°)
เลือกทางที่เข้าใกล้ goal ตาม mask ภาพ และใช้ Pure Pursuit ตามเส้นเดียวกับภาพ
ไม่เพิ่มการหมุนอยู่กับที่หรือถอยหลัง และไม่เปิด LiDAR obstacle stop กลับมา

arc อ้างอิงวงเลี้ยวจากตัวหุ่นแล้วตรวจส่วนที่มองเห็นด้านหน้า จึงไม่บังคับให้เริ่มที่ Y=0
หากยืนยันว่าจะเลี้ยวตามกลุ่มทางด้านข้าง ข้อมูล unknown ใน near field X<ระยะเริ่มที่กล้องเห็น
ใช้สมมติฐานผู้คุมของ image_only; image veto ที่ตรวจพบจริงไม่ถูกเปลี่ยนเป็น free
บริเวณไกลกว่า near field ยังต้องอยู่ใน mask ที่รองรับทั้งความกว้าง

**profile เดิมหมุนสะสมได้เพียง 0.35 rad (~20°)** จึงไม่พอเลี้ยวแยก 90°
เพิ่ม preset แยกชื่อ turning session: 0.15 m/s, 0.15 rad/s, 100 m, 900 s,
หมุนสะสมไม่เกิน 2π rad (~360°) ต่อรอบ โดยไม่เปลี่ยน profile เก่า
ต้องอัปเดตทั้ง ROS2 และ shared core บนหุ่นให้รู้จัก preset นี้ก่อนใช้

## 1. คอม: หยุด session/field launch เดิม แล้วอัปเดต

```bash
cd ~/husk-live
git pull --ff-only origin experiment/local-ai-lidar
python3 tools/create_ros2_workspace.py --workspace "$HOME/husky_vision_ws" --update-code
source /opt/ros/jazzy/setup.bash
source "$HOME/husky_vision_ws/.venv/bin/activate"
cd ~/husky_vision_ws
python /usr/bin/colcon build --symlink-install --packages-select husky_vision
source install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run husky_vision prepare_operator_profile \
  --profile "$HOME/husky_vision_ws/robot/robot-nojoy-015.json" \
  --output "$HOME/husky_vision_ws/robot/robot-nojoy-junction.json" \
  --robot-output "$HOME/husky_vision_ws/robot/ros1-nojoy-junction.json" \
  --supervised-extended-goal --extended-session-sec 900 --turning-session

scp -r "$HOME/husky_vision_ws/src/husky_vision/ros1/husky_vision_bridge" \
  administrator@10.82.13.0:~/husky_ws/src/
scp "$HOME/husky_vision_ws/robot/ros1-nojoy-junction.json" \
  administrator@10.82.13.0:~/husky-nojoy-junction.json
```

ใช้ IP ที่เชื่อมอยู่จริงหากเปลี่ยนไปแล้ว เครื่องมือไม่เขียนทับ profile ที่มีอยู่
สร้างไฟล์ครั้งแรกเท่านั้น; รอบต่อไปใช้ไฟล์ที่สร้างแล้ว

## 2. หุ่น: ปิด bridge/console เก่า แล้ว build

```bash
source /opt/ros/noetic/setup.bash
cd ~/husky_ws
catkin_make --pkg husky_vision_bridge
source devel/setup.bash
roslaunch husky_vision_bridge bridge.launch \
  profile_path:="$HOME/husky-nojoy-junction.json" \
  odom_topic:=/odometry/filtered scan_topic:=/scan base_frame:=base_link \
  allow_robot_output:=true supervised_trial:=true
```

tunnel เดิมใช้ต่อได้ console อีก terminal ใช้ profile ใหม่ด้วย:

```bash
rosrun husky_vision_bridge operator_console.py \
  _profile_path:="$HOME/husky-nojoy-junction.json" \
  _allow_robot_output:=true _supervised_trial:=true _session_sec:=900.0
```

## 3. คอม: เปิดใหม่ด้วย profile junction

```bash
QT_QPA_PLATFORM=xcb ros2 launch husky_vision vision_field.launch.py \
  profile:="$HOME/husky_vision_ws/robot/robot-nojoy-junction.json" \
  map:="$HOME/husky_vision_ws/maps/outdoor1.yaml" \
  guidance_mode:=image_only supervised_trial:=true \
  assume_near_field_clear:=false allow_motion:=true
```

ตั้ง pose ให้ตรง แล้วปัก goal ในแขนทางแยกที่ต้องการ
reset/enable และกด g แบบเดิม ถ้าทางเลี้ยวผ่านเกณฑ์ status จะมี
`reference_kind=junction_arc` และ `terminal_heading_rad` เป็นมุมเลี้ยวของปลายเส้น
goal ใหม่จะยกเลิก reference เก่าเพื่อสร้างทางตามพิกัดใหม่

## ผลที่ตรวจได้

มีการจำลองแบบป้อนกลับจาก mask รูปทางแยกกว้าง 1.5 m: goal (2,3) และ (2,-3)
ทั้งซ้ายและขวาถึง tolerance 0.15 m พร้อม heading ปลายมากกว่า 90° ตามแขนทาง
เป็น ideal image labels และ kinematic robot ไม่ใช่การวิ่งจริงหรือการรับรองการเห็นทางของกล้องจริง
หากกล้องไม่เห็นแขนทางหรือ mask ตัดทางนั้นขาด ยังอาจสร้างเส้นเลี้ยวไม่ได้
