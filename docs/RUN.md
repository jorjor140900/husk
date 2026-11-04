# วิธีรัน camera assist


## 1. Clone และ export source บน laptop

```bash
git clone --branch submission/camera-steering-assist --single-branch \
  https://github.com/jorjor140900/husk.git ~/husk-submission
cd ~/husk-submission
python3 tools/create_ros2_workspace.py --workspace "$HOME/husky_submission_ws"
```

Source ใน Git ใช้ชื่อ package `husky_drivable_area` เครื่องมือ export เปลี่ยนชื่อเป็น `husky_vision` เหมือนเครื่องทดลอง คำสั่งต่อจากนี้ใช้ชื่อที่ export แล้ว

กรณีอัปเดต source ที่ export แล้ว ใช้ `--update-code` (สำรอง package เดิมใต้ `.source-backups` ก่อนแทนที่) ไม่มีการรัน build/test/ROS โดยเครื่องมือ export

## 2. Dependencies และไฟล์จากเครื่องทดลอง

ต้องมี ROS 2 Jazzy, colcon และ dependencies ตาม `package.xml` ติดตั้งส่วน ROS ที่ใช้หลัก ๆ:

```bash
sudo apt install python3-venv python3-colcon-common-extensions \
  ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-rqt-image-view \
  ros-jazzy-compressed-image-transport ros-jazzy-tf2-ros-py \
  ros-jazzy-rosidl-runtime-py
cd ~/husky_submission_ws
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r ~/husk-submission/requirements-ros2.txt
```

camera-assist launch ใช้ CUDA สำหรับ inference ต้องมี PyTorch/CUDA ที่ตรง GPU หากเครื่องทดลองมี environment ที่ใช้งานได้แล้วให้ใช้รุ่น dependencies เดิมจากเครื่องนั้น ไม่ได้กำหนด CUDA driver/version ใหม่ใน Git นี้

ไฟล์จริงที่ต้องมี:

1. โฟลเดอร์โมเดล SegFormer พร้อม weights, model config และ processor config
2. Camera calibration YAML ที่ตรงกล้อง/ความละเอียดภาพ
3. Camera mount JSON ที่วัด extrinsic จริง
4. Robot profile ที่ระบุ model/calibration/mount paths, frames, footprint และ permission wiring

สำหรับผู้จัดทำที่มี workspace เดิม ใช้โปรไฟล์ล่าสุดได้:

```bash
cp ~/husky_vision_ws/robot/robot-camera-assist-035.json ~/husky_submission_ws/robot/
cp ~/husky_vision_ws/robot/ros1-camera-assist-035.json ~/husky_submission_ws/robot/
```

การ copy profile ไม่ได้ copy weights/calibration ที่มันอ้างถึง ตรวจว่า absolute paths ใน profile ยังมีอยู่ ถ้าขนไปเครื่องใหม่ต้องนำไฟล์จริงไปด้วยและแก้ path ให้ถูกต้อง ตัวอย่าง `config/robot.example.json` และ `camera_mount.example.json` มีค่า `null` โดยตั้งใจ ต้องกรอกจากข้อมูลวัดจริงก่อนใช้ ไม่ใช้เป็น motion profile ทันที


```bash
ros2 run husky_vision prepare_operator_profile \
  --profile "$HOME/husky_vision_ws/robot/robot-camera-assist-018.json" \
  --output "$HOME/husky_submission_ws/robot/robot-camera-assist-035.json" \
  --robot-output "$HOME/husky_submission_ws/robot/ros1-camera-assist-035.json" \
  --supervised-extended-goal --extended-session-sec 900 \
  --turning-session --camera-assist --camera-assist-speed 0.35
```

ชื่อ flag extended-goal เป็น preset session ที่ใช้ร่วมกัน ไม่ได้เปิด goal navigation ใน branch นี้

## 3. Build บน laptop

```bash
cd ~/husky_submission_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
python /usr/bin/colcon build --symlink-install --packages-select husky_vision
source install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## 4. ดูภาพโดยไม่เชื่อมหุ่น

```bash
ros2 launch husky_vision camera_assist.launch.py \
  profile:="$HOME/husky_submission_ws/robot/robot-camera-assist-035.json" \
  allow_motion:=false
```

ถ้าอุปกรณ์ไม่ตรงกับชื่อ webcam เดิม เพิ่ม `camera_device:=/dev/videoN` ที่เป็นกล้องจริง; การเปลี่ยนรุ่นกล้อง/ตำแหน่งติดตั้งต้องใช้ calibration/mount ที่ตรงกัน

อีก terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/husky_submission_ws/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run rqt_image_view rqt_image_view /husky_assist/overlay
```

เลือก `/husky_assist/overlay/compressed` โหมด `allow_motion=false` แสดง preview ไม่ส่งคำสั่งหุ่น

## 5. เตรียม package บนหุ่นครั้งแรก/หลังอัปเดต

หยุดหุ่นและปิด launch/console/teleop ชุดเก่าก่อน ไม่เปิดสองชุดควบคุมพร้อมกัน ตัวอย่างใช้ IP `10.82.13.0` ให้แทนด้วย IP ปัจจุบันทุกจุด

บน laptop:

```bash
scp -r ~/husky_submission_ws/src/husky_vision/ros1/husky_vision_bridge \
  administrator@10.82.13.0:~/husky_ws/src/
scp ~/husky_submission_ws/robot/ros1-camera-assist-035.json \
  administrator@10.82.13.0:~/husky-camera-assist-035.json
```

SSH เข้าหุ่นแล้ว build (ต้องมี rosbridge_server ใน ROS1):

```bash
ssh administrator@10.82.13.0
source /opt/ros/noetic/setup.bash
cd ~/husky_ws
catkin_make --pkg husky_vision_bridge
source devel/setup.bash
```

Base driver, `/odometry/filtered` และ TF ของหุ่นต้องทำงานตาม bringup ปกติ การไม่ใช้ LiDAR วางทางไม่ได้หมายความว่าไม่ต้องมี odometry/TF

## 6. เปิดระบบจริงทุก terminal

### T1 — Laptop: SSH tunnel

```bash
ssh -N -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:9090:127.0.0.1:9090 administrator@10.82.13.0
```

ค้างไว้ ไม่มี output เป็นปกติ ถ้ามี tunnel เดิมที่ใช้งานอยู่แล้วไม่ต้องเปิดซ้ำ

### T2 — SSH เข้าหุ่น: gateway

```bash
ssh administrator@10.82.13.0
source /opt/ros/noetic/setup.bash
source ~/husky_ws/devel/setup.bash
roslaunch husky_vision_bridge bridge.launch \
  profile_path:="$HOME/husky-camera-assist-035.json" \
  odom_topic:=/odometry/filtered base_frame:=base_link \
  allow_robot_output:=true supervised_trial:=true
```

### T3 — SSH เข้าหุ่น: operator console

```bash
ssh -t administrator@10.82.13.0
source /opt/ros/noetic/setup.bash
source ~/husky_ws/devel/setup.bash
rosrun husky_vision_bridge operator_console.py \
  _profile_path:="$HOME/husky-camera-assist-035.json" \
  _allow_robot_output:=true _supervised_trial:=true _session_sec:=900.0
```

คง STOPPED ไว้ก่อน ยังไม่กด g; Space ที่ terminal นี้ตัด permission

### T4 — Laptop: camera assist

```bash
source /opt/ros/jazzy/setup.bash
source ~/husky_submission_ws/.venv/bin/activate
source ~/husky_submission_ws/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch husky_vision camera_assist.launch.py \
  profile:="$HOME/husky_submission_ws/robot/robot-camera-assist-035.json" \
  allow_motion:=true
```

### T5 — Laptop: ภาพ

```bash
source /opt/ros/jazzy/setup.bash
source ~/husky_submission_ws/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run rqt_image_view rqt_image_view /husky_assist/overlay
```

### T6 — Laptop: reset/enable

```bash
source /opt/ros/jazzy/setup.bash
source ~/husky_submission_ws/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 service call /husky_assist/reset_trial std_srvs/srv/Trigger '{}'
ros2 service call /husky_assist/set_enabled std_srvs/srv/SetBool '{data: true}'
ros2 service call /husky_link/set_enabled std_srvs/srv/SetBool '{data: true}'
```

ต้องได้ success ทั้งสามคำสั่ง จากนั้นกลับ T3 กด g ภาษาอังกฤษ ไม่ต้อง Enter เพื่ออนุญาต session

### T7 — Laptop: keyboard teleop

```bash
source /opt/ros/jazzy/setup.bash
source ~/husky_submission_ws/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run husky_vision camera_assist_teleop
```

โฟกัส T7 แล้วกด i/w/Up ค้างเพื่อเดินและให้ระบบเลี้ยวตามเส้นเหลือง Space/k ส่งศูนย์ทันที การปล่อยปุ่ม terminal ใช้ key-repeat timeout ประมาณ .6 s กลับมากดเดินได้ตราบใดที่ข้อมูล/permission/session ยังใช้ได้

จบการทดลอง: กด Space ที่ T7 แล้ว Space ที่ T3 จากนั้นปิด launch/bridge/tunnel หุ่นไม่จำเป็นต้องวิ่งครบ 900 s; permission/network/odometry หรือ budget ระยะ/มุมทำให้หยุดก่อนเวลาได้

## 7. สถานะประกอบการอ่านผล

จาก terminal ที่ source ROS2/workspace แล้ว:

```bash
ros2 topic echo /husky_assist/status --once --full-length
ros2 topic echo /husky_control/safety/status --once --full-length
ros2 topic echo /husky_link/status --once --full-length
ros2 topic echo /drivable_area/status --once --full-length
```

แยก `vision_paused` ออกจาก `trial_fault` และ `receiver.armed` เส้นเหลืองบนภาพอย่างเดียวไม่ยืนยันว่าคำสั่งมอเตอร์ได้รับอนุญาต ข้อมูลเก่าหรือ fault ทางการสื่อสารยังต้องหยุด ไม่กด enable ซ้ำเพื่อข้ามสาเหตุโดยไม่ตรวจสถานะ
