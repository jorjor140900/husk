# Husky A200 — Camera Steering Assist

**ระบบตรวจจับพื้นที่ขับเคลื่อนและช่วยควบคุมทิศทางจากภาพกล้อง**

ผู้ใช้กดเดินหน้าค้างบน keyboard teleop ระบบตรวจพื้นที่จากภาพและช่วยเลี้ยวหุ่นให้เข้าแนวกลางทาง ปล่อยปุ่มแล้วหยุด ไม่มีการปัก goal หรือวิ่งไปจุดหมายโดยอัตโนมัติ

## ลำดับการทำงาน

```mermaid
flowchart LR
  A[USB Webcam บน laptop] --> B[SegFormer]
  B --> C[Probability filtering และ drivable mask]
  C --> D[Calibration: mask เป็นพิกัดพื้น]
  D --> E[แนวกลางและ Dijkstra: เส้นเหลือง]
  E --> F[Pure Pursuit และ heading feedback]
  K[กดเดินหน้าค้าง] --> F
  O[Odometry และ TF] --> F
  F --> G[Motion permission และตัวกรองคำสั่ง]
  G --> H[ROS2 bridge / rosbridge ผ่าน SSH tunnel]
  H --> I[ROS1 gateway / watchdog บน Husky]
  I --> J[คำสั่งขับเคลื่อน]
```

- **Laptop:** Ubuntu / ROS 2 Jazzy, Cyclone DDS, กล้อง, SegFormer, planner, controller
- **Husky:** ROS 1 Noetic, base driver/odometry, operator console, gateway
- **สีเขียว:** mask ที่ระบบจำแนกว่าเป็นพื้นที่ขับได้
- **สีเหลือง:** reference ที่ผ่านการตรวจพื้นที่สำหรับ controller
- **สีน้ำเงิน:** แกนหุ่น `Y=0` ฉายตาม calibration และตำแหน่งกล้อง ไม่จำเป็นต้องตรงกลางภาพ


| ส่วน | ไฟล์หลัก |
|---|---|
| โหลดโมเดลและ inference | [road_navigation/segmentation.py](road_navigation/segmentation.py) |
| กรอง mask, optical flow, hysteresis, ช่องเล็ก | [road_navigation/detector.py](road_navigation/detector.py) |
| รับภาพและรัน inference ใน ROS 2 | [husky_drivable_area/nodes.py](husky_drivable_area/nodes.py) |
| พิกัดพื้นและตำแหน่งกล้อง | [ground_geometry.py](husky_drivable_area/ground_geometry.py), [assist_geometry.py](husky_drivable_area/assist_geometry.py) |
| หาแนวกลางและทางต่อใน mask | [area_reference.py](husky_drivable_area/area_reference.py) |
| Pure Pursuit | [control.py](husky_drivable_area/control.py) |
| วงรอบควบคุม, heading feedback, reference hold | [camera_assist_node.py](husky_drivable_area/camera_assist_node.py), [reference_stability.py](husky_drivable_area/reference_stability.py) |
| วาดเส้นเหลือง/น้ำเงิน | [assist_overlay.py](husky_drivable_area/assist_overlay.py) |
| กดเดินหน้าค้าง | [assist_teleop.py](husky_drivable_area/assist_teleop.py) |
| กรองคำสั่งและตรวจ permission | [safety_node.py](husky_drivable_area/safety_node.py) |
| เชื่อม ROS 2–ROS 1 | [robot_bridge_node.py](husky_drivable_area/robot_bridge_node.py), [robot_link.py](husky_drivable_area/robot_link.py) |
| Watchdog และ budget ของ session | [husky_link_core](husky_link_core) |
| Gateway และ console บนหุ่น | [ros1/husky_vision_bridge](ros1/husky_vision_bridge) |
| เปิดระบบ | [camera_assist.launch.py](launch/camera_assist.launch.py) |
| เตรียมข้อมูล/เทรน | [tools/prepare_cvat_dataset.py](tools/prepare_cvat_dataset.py), [tools/train_segmentation.py](tools/train_segmentation.py) |

## ค่าที่กำหนด

| รายการ | ค่า |
|---|---:|
| ความเร็วเดินหน้าสูงสุด | 0.35 m/s |
| ความเร็วเชิงมุมสูงสุด | 0.15 rad/s |
| Pursuit lookahead | 0.35 m |
| Grid ของ planner | 0.05 m |
| อายุ reference สูงสุด | 2 s จาก capture เดิม |
| เพดานความเร็ว reference เก่าเกิน 0.5 s / held reference | 0.08 m/s |
| Session | สูงสุด 900 s, 100 m, มุมสะสม 2π |
| Model input ใน camera-assist launch | 256 × 160 |
