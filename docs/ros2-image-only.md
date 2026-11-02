# โหมดเดินตามเส้นจากภาพและ AI โดยไม่ใช้ LiDAR หยุดสิ่งกีดขวาง

เพิ่ม `guidance_mode:=image_only` บน branch `experiment/local-ai-lidar`
ใช้ได้เมื่อเลือก `supervised_trial:=true` และ profile supervised ที่ตรงกันเท่านั้น
เลือกโหมดนี้ตอนเปิด launch; ไม่เปลี่ยนโหมดกลาง session

## การทำงานที่เปลี่ยน

- รับ goal จาก RViz โดยตรง ไม่รอ global route และไม่เปิด global planner/costmap
- สร้าง metric reference จาก segmentation ของกล้อง ทิศ goal และ calibration/mount
- ใช้ Pure Pursuit คอนโทรลตาม reference เดียวกับเส้นเหลืองบนภาพ
- เลือก candidate ที่ปลายเส้นเข้าใกล้ local goal มากที่สุดก่อน แล้วใช้ความอยู่กลางทาง/ความโค้งตัดสินกรณีระยะเท่ากัน
  ไม่ให้การอยู่กลางกลุ่มสีเขียวชนะทิศ goal เหมือนคะแนนเดิม
- เมื่อ goal อยู่ในช่วงที่สร้าง reference ได้ จำกัดปลาย X ไม่ให้เลย goal
  เส้นยังเป็น reference ระยะใกล้และไม่ลากผ่าน mask ที่ไม่รองรับเพื่อเชื่อมถึง goal
- vision controller ไม่ subscribe LaserScan ไม่ fuse จุด LiDAR ไม่ตรวจ LiDAR footprint หรือ stop sweep
- ชั้น supervised ปลายทางใช้ trial/permission gate เดิม ไม่เรียก LiDAR collision filter
- จุดสิ่งกีดขวาง ระยะใกล้ `+inf` หรือ scan ขาด ไม่ถูกใช้ตัดสินใจหยุดในตัวคอนโทรลภาพ
- ไม่เปิด vision veto costmap node ที่ไม่ได้ใช้ และซ่อน costmap ที่ไม่ได้ใช้ใน RViz

LiDAR ยังส่งให้ AMCL ระบุตำแหน่งได้ การที่ LiDAR ไม่ใช้หยุดสิ่งกีดขวางไม่ได้ตัดความต้องการ
localization/TF: ถ้า AMCL ไม่มีตำแหน่งสด ระบบยังหยุดด้วยเหตุ localization
ภาพที่ไม่มีทางใน mask, ภาพเก่า, goal อยู่ด้านหลัง, goal ถึงแล้ว, TF/odom ขาด,
permission/session หมด และ watchdog ยังทำให้คำสั่งเป็นศูนย์
ขอบเขตตัวหุ่นและ side margin ของการเลือกเส้นในภาพยังอ่านจาก profile เดิม

ไม่มีการตรวจสิ่งกีดขวางจาก LiDAR มายับยั้งคำสั่งในโหมดนี้
หาก AI เห็นวัตถุเป็นพื้นที่ผ่านได้ ระบบอาจสร้างทางผ่านบริเวณนั้นได้ ผู้คุมเป็นผู้กำกับการเคลื่อนที่

## อัปเดตฝั่งคอม

ปิด field launch เดิมก่อน แล้วใช้:

```bash
cd ~/husk-live
git switch experiment/local-ai-lidar
git pull --ff-only origin experiment/local-ai-lidar
python3 tools/create_ros2_workspace.py \
  --workspace "$HOME/husky_vision_ws" --update-code

source /opt/ros/jazzy/setup.bash
source "$HOME/husky_vision_ws/.venv/bin/activate"
cd ~/husky_vision_ws
python /usr/bin/colcon build --symlink-install --packages-select husky_vision
source install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

ไม่ต้องแก้ ROS1 gateway หรือ operator console เพื่อเปลี่ยน guidance mode
ใช้ bridge/tunnel และ profile บนหุ่นชุดเดียวกับ supervised 0.15 m/s เดิม

## เปิดระบบ

บนคอม ใช้ profile long goal เดิม 0.15 m/s, ระยะสำรอง 100 m และเวลา 900 s:

```bash
QT_QPA_PLATFORM=xcb ros2 launch husky_vision vision_field.launch.py \
  profile:="$HOME/husky_vision_ws/robot/robot-nojoy-015.json" \
  map:="$HOME/husky_vision_ws/maps/outdoor1.yaml" \
  guidance_mode:=image_only \
  supervised_trial:=true \
  assume_near_field_clear:=false \
  allow_motion:=true
```

หากชื่อ profile ของเครื่องต่างออกไป ให้ใช้ไฟล์ supervised เดิมที่เคยส่งคำสั่งได้
near-field assumption ใช้เฉพาะ local_fusion จึงให้ปิดใน image_only
สำหรับดูภาพ/เส้นโดยไม่อนุญาตคำสั่ง เปลี่ยน `allow_motion:=false`

เปิดภาพจาก terminal ที่ source environment เดียวกัน:

```bash
QT_QPA_PLATFORM=xcb ros2 run image_view image_view --ros-args \
  -r image:=/husky_nav/vision_overlay -p image_transport:=compressed
```

ตั้ง 2D Pose Estimate และ goal ด้านหน้าใน RViz
ภาพต้องขึ้น `IMAGE AI ONLY` และ `LiDAR obstacle stops OFF`
goal status เป็น `local_goal_ready`; ไม่ต้องรอ Nav2 สร้าง global path

ก่อนเริ่มรอบ บนคอมใช้ขั้นตอน reset/enable เดิมขณะที่ operator ยัง STOPPED:

```bash
ros2 service call /husky_nav/reset_trial std_srvs/srv/Trigger "{}"
ros2 service call /husky_nav/set_enabled std_srvs/srv/SetBool "{data: true}"
ros2 service call /husky_link/set_enabled std_srvs/srv/SetBool "{data: true}"
```

บนหุ่น ใช้ console เดิมและกด g หลัง enable link แล้ว:

```bash
rosrun husky_vision_bridge operator_console.py \
  _profile_path:="$HOME/husky-nojoy-015.json" \
  _allow_robot_output:=true _supervised_trial:=true _session_sec:=900.0
```

Space หยุด และ q หยุดแล้วออก ตัว console ไม่ส่งคำสั่งเดิน/เลี้ยวเอง
เพดานมุมหมุนสะสม 0.35 rad และเพดานอื่นใน profile ยังมีผล
หาก session หยุดแล้ว ให้เริ่มรอบใหม่ด้วยขั้นตอน reset/enable ตามเดิม

## ตรวจว่าเป็นผลคอนโทรลจริง

```bash
ros2 topic echo /husky_nav/vision_control_status --once --full-length
```

ควรมีค่าต่อไปนี้:

```json
{
  "guidance_mode": "image_only",
  "steering_source": "image_ai",
  "map_use": "goal_position_only",
  "lidar_obstacle_checks_enabled": false,
  "global_plan_required": false,
  "lidar_reference_footprint_checked": false,
  "lidar_stop_sweep_required": false
}
```

บางฟิลด์รายละเอียดจะเกิดหลังสร้าง reference สำเร็จ ส่วน guidance_mode และ lidar_obstacle_checks_enabled
มีตั้งแต่ status ปกติ การเห็นเส้นเหลืองยังแยกจากการเริ่ม permission session

รุ่นที่เลือก goal ก่อนรายงาน `goal_priority=true`, `guide_target_ground_xy`,
`reference_end_ground_xy` และ `reference_end_to_target_m` เพื่อเทียบเป้าท้องถิ่นกับปลาย reference
พิกัดเหล่านี้อยู่ในกรอบพื้น ณ เวลา capture ไม่ใช่พิกัด map ใน RViz
goal ที่ไกลกว่า lookahead จะให้ทิศทางสำหรับ reference ระยะใกล้ ไม่จำเป็นต้องเห็นปลายเส้นถึง goal ไกลในภาพเดียว

## ขอบเขตผลทดสอบ

ทดสอบ native ROS2 ด้วยภาพ/mask, TF ล่าช้า 90 ms และ odometry จำลองใน container ที่ตัด network
ตรวจว่า controller ไม่มี scan subscriber, ยังมีคำสั่งเดินเมื่อมีจุด LiDAR ใกล้,
เมื่อ scan ขาด และเมื่อ scan เป็น unknown พร้อมตรวจว่าภาพ veto/เก่าและ cancel ยังหยุด
ทดสอบการถึง goal จำลอง 2 m โดย preview gate ยังปิดและไม่มี publisher ไป `/cmd_vel`
ผลนี้ไม่ใช่การทดลองวิ่งหรือระยะหยุดของหุ่นจริง

กลับไปตรวจ LiDAR แบบเดิมได้โดยเปิดใหม่ด้วย `guidance_mode:=local_fusion`
ส่วน default ที่ไม่ระบุ argument ยังเป็น `global_route`
