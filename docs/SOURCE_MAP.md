# ที่มาของ source และการจัด branch

ฐานของ branch นี้: `7ae63a1` บน `experiment/local-ai-lidar` — รุ่น camera assist 0.35 m/s วันที่ 4 พฤศจิกายน 2569 Branch พัฒนาเดิมและประวัติไม่ได้ถูกลบหรือ force-push

## สิ่งที่คงไว้

- SegFormer inference, filtering, ground projection, centreline/Dijkstra, pursuit/heading control, reference continuity และ keyboard teleop
- Permission, ROS2 bridge, ROS1 gateway/operator console, lease/watchdog และ session budgets
- Configuration ที่ camera assist โหลดจริง รวมถึง template สำหรับ measured robot/mount
- เครื่องมือ CVAT dataset preparation และ SegFormer training ที่เกี่ยวข้องกับโมเดล

## สิ่งที่ตัดออกจากหน้าไฟล์ปัจจุบัน

Legacy detector entrypoints, Nav2/SLAM/goal/waypoint nodes และ launch, LiDAR demos/replay tools, simulators/self-test entrypoints, report exports/figures, การทดลองเก่า, notebook ซ้ำและชุด tests เก่า รวมถึง ROS1 webcam launch (กล้องล่าสุดอยู่บน laptop)

## การย้าย helper โดยคงสูตรเดิม

| ไฟล์เดิม | ไฟล์ใน branch ส่งงาน | ส่วนที่นำมาใช้ |
|---|---|---|
| `vision_goal.py` | `assist_geometry.py` | `VisionReferenceConfig`, `sample_ground_labels` |
| `nav2_camera_path.py` | `image_projection.py` | การฉาย metric path ลงภาพ เปลี่ยนชื่อเป็น `project_reference_path` |
| `vision_goal_overlay.py` | `assist_overlay.py` | overlay เฉพาะ assist; ไม่มี edge fits หรือ banner โหมดเก่า |
| `controller_node.py` | `timestamps.py` | `stamp_ns` |
| `vision_grid.py` | `vision_grid.py` | เฉพาะ encode/decode segmentation packet |
| `reference_stability.py` | `reference_stability.py` | เฉพาะการใช้ reference เดิมหลังตรวจ mask |

`nodes.py` ตัด route subscribers และ display worker ของ Nav2/legacy ที่ไม่ได้เปิดใช้ใน camera-assist launch ส่วน `detector.py` เหลือ SegFormer probability pipeline ไม่มี pixel-path planner หรือ appearance fallback จุดตัดสินใจของโหมดล่าสุดคงไว้ เปลี่ยนชื่อ status `nav2_segmentation_only` เป็น `segmentation_only` ให้ตรงหน้าที่

`safety_node.py`, `collision.py` และ bridge/protocol เป็น utility ที่ระบบจริงใช้ร่วมกัน จึงคง schema/การตรวจสอบและฟังก์ชันสนับสนุนไว้ แม้บาง telemetry เช่น scan/cloud ไม่ได้ใช้วางเส้นในโหมดนี้ safety entrypoint ใน branch ส่งงานรับเฉพาะ supervised mode และไม่ subscribe scan

Profile loader ไม่ตรวจส่วน navigation เก่าที่ไม่ได้ใช้งานใน camera assist; ยังคงตรวจ geometry, calibration, permission และ motion preset ตามเดิม

`setup.py` เหลือ 8 entrypoints ที่ใช้งานจริง `package.xml` ไม่พึ่ง Nav2/SLAM/RViz เครื่องมือ export สร้าง package ชื่อ `husky_vision` เหมือนเครื่องทดลอง สำรอง source package ก่อน update และไม่เปลี่ยน weights/calibration/profile ภายนอก package

## สถานะการตรวจทาน

ตรวจโครงสร้างไฟล์ Python/JSON/XML, import ภายใน repository, entrypoint/launch reference และ diff ของ source โดยไม่รันโปรแกรม ROS, inference, unit tests หรือทดลองกับหุ่นตามคำขอผู้จัดทำ การจัดชุด source ไม่ใช่ผลทดสอบซ้ำบน hardware และไม่เพิ่มข้ออ้างความแม่นยำในรายงาน
