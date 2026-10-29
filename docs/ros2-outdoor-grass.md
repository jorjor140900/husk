# ทดลองตัดหญ้าสีเขียวออกจากพื้นที่วิ่ง

โมเดล binary SegFormer อาจนับหญ้าเป็น drivable จากภาพบางแสง แม้หญ้าอีกฝั่งจะไม่ผ่าน
แมพ LiDAR ไม่ได้ระบุว่า grass/ทางเดิน และเส้นเหลืองในภาพไม่ได้รับรองพื้นที่ปลอดภัย
การเพิ่ม probability threshold อย่างเดียวไม่แก้กรณีโมเดลให้หญ้าคะแนนสูง

ชุดนี้เพิ่มตัวกรอง **สีเขียวจากภาพ BGR ดิบ** เป็น veto เสริม ไม่ใช่โมเดลจำแนกหญ้าใหม่
ใช้ HSV ร่วมกับ excess green แบบ normalized และขยาย veto 2 พิกเซลที่ภาพดิบ
ประมวลผลทั้งซ้ายและขวา ไม่กำหนดว่าฝั่งไหนต้องเป็นหญ้า
ใช้ก่อนรวม temporal history และบังคับตัดซ้ำหลังรวม จึงไม่คืนหญ้าจากเฟรมเก่า
ผลมีผลต่อ raw mask, seed-connected road mask, path, clearance และ segmentation packet
ที่ฉายเข้า AI costmap ของ Nav2 ไม่ได้เปลี่ยนเพียงภาพ overlay

## อัปเดต

ในโฟลเดอร์ Git เดิม:

```bash
git pull --ff-only origin work
python3 tools/create_ros2_workspace.py \
  --workspace ~/husky_vision_ws --update-code --update-detector
```

ต้องมี `--update-detector` เพราะปกติ updater เก็บโค้ด detector ใน workspace ไว้
flag นี้สำรอง detector/CLI เดิมก่อนเขียน detector, CLI และโมดูล vegetation ใหม่
ไม่เปลี่ยน visualization/ขนาดเส้น, โมเดล, calibration, mount หรือค่าที่วัดใน robot profile

หยุดเฉพาะ AI เดิม แล้ว build:

```bash
cd ~/husky_vision_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
python /usr/bin/colcon build --symlink-install --packages-select husky_vision
source install/local_setup.bash
```

คง webcam ที่ส่ง `/camera/image_raw` เปิดอยู่ แล้วเปิด AI:

```bash
ros2 launch husky_vision drivable_area.launch.py \
  use_webcam:=false image_topic:=/camera/image_raw \
  model_path:="$HOME/husky_vision_ws/models/husky-segformer" \
  device:=cpu model_width:=256 model_height:=160 cpu_threads:=2 \
  config_file:="$HOME/husky_vision_ws/src/husky_vision/config/outdoor-no-grass.json" \
  camera_calibration:="$HOME/husky_vision_ws/camera_calibration/camera-active.yaml" \
  camera_mount:="$HOME/husky_vision_ws/robot/robot-mount.json"
```

บรรทัด Ready ต้องมี `green vegetation veto=True`
status มี `vegetation_veto.enabled=true`, จำนวนพิกเซลที่ตัด และ
`verified_grass_classification=false` เพื่อระบุขอบเขตของวิธีนี้ตามจริง
Nav2 รับ segmentation packet จาก topic เดิม ไม่ต้องเปลี่ยน costmap เพื่อเปิดตัวกรองนี้
คง `allow_motion:=false` ขณะตรวจและเทียบภาพ

## ตรวจภาพก่อนใช้

จอดหุ่นแล้วดูภาพดิบกับ overlay ในสถานการณ์เดียวกัน อย่าใช้ภาพ overlay ที่ถูกย้อมเขียว
เพื่อเลือกเกณฑ์สี เปิด viewer ทีละภาพเพื่อลดภาระ CPU:

```bash
QT_QPA_PLATFORM=xcb ros2 run image_view image_view --ros-args \
  -r image:=/drivable_area/vegetation_veto -p image_transport:=raw \
  -p autosize:=false -p width:=960 -p height:=720
```

สีขาวคือถูก veto สีดำคือไม่มี veto สีเพิ่ม **ไม่ได้หมายความว่าขับได้**
หลังดู mask เปลี่ยน viewer เป็น overlay:

```bash
QT_QPA_PLATFORM=xcb ros2 run image_view image_view --ros-args \
  -r image:=/drivable_area/overlay -p image_transport:=compressed \
  -p autosize:=false -p width:=960 -p height:=720
```

ตรวจหญ้าทั้งสองฝั่ง แดด/เงา และช่วงหญ้าบุกรอยต่อทางเดิน ดูว่าตัดหญ้าโดยไม่ตัด
พื้นทางเดินจนไม่มีทางอ้างอิง ห้ามใช้ผลทดสอบสีสังเคราะห์เป็นหลักฐานว่าภาคสนามผ่าน
ตัวกรองอาจตัดสีเขียวอื่น เช่นพื้นทาสี และอาจพลาดหญ้าแห้ง/เหลือง/มืดมากหรือถูกบัง
เกณฑ์เริ่มต้นใน `outdoor-no-grass.json` ยังต้องตรวจภาพดิบจริงของพื้นที่
หากไม่มีทางที่ผ่าน veto ระบบต้องรายงานไม่มี path แทนการลดข้อห้ามเพื่อให้วิ่งต่อ

ระยะ 2 พิกเซลเป็นเพียงการขยาย veto ในภาพ ไม่ใช่ระยะเผื่อขนาดหุ่นหน่วยเมตร
footprint/inflation, ความสดข้อมูล, localization, LiDAR และ permission gate ยังต้องผ่านแยกกัน
ตัวกรองนี้ไม่ทำให้ส่วน VLP16 runtime หรือการขับอัตโนมัติที่ยังค้างกลายเป็นพร้อมใช้

หากต้องคืนพฤติกรรมเดิม ให้หยุด AI และเปิดใหม่โดยใช้ config เดิม
ค่าเริ่มต้นของ `reject_green_vegetation` เป็น false ไม่ต้องเปลี่ยนโมเดลกลับ

## ระยะยาว

เก็บภาพดิบของทางจริงจากตำแหน่งกล้องเดิม ทั้งแดด เงา ทางแยก และหญ้าทั้งสองด้าน
label หญ้าเป็น non-drivable และทางเดินเป็น drivable แล้ว fine-tune โมเดล
แยก validation ตามช่วง/บริเวณการเดิน ไม่ใช้เฟรมติดกันเป็นหลักฐานทดสอบอิสระ
ประเมิน false drivable บนหญ้าแยกจาก pixel accuracy รวม เพราะพื้นที่ทางเดินมากอาจกลบความผิดพลาด

ทดสอบอัตโนมัติครอบคลุม veto ทับผล AI ความมั่นใจสูง, temporal history,
การ downsample, blocked mask อื่น, packet ไป Nav2 และการ backup ตอนส่งออก
บน host ทดสอบ ตัวกรองอย่างเดียวที่ภาพสังเคราะห์ 640×480 ใช้ประมาณ 4 ms;
ยังต้องวัด latency จริงบนคอมผู้ใช้และตรวจอายุผลตามเกณฑ์ควบคุมเดิม
